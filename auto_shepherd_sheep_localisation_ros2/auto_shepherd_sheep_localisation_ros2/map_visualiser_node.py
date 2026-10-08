#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, Image
from nav_msgs.msg import Path
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Empty
from cv_bridge import CvBridge
import cv2
import math
import json
import re
import time

try:
    import pynvml
except ImportError:
    pynvml = None

import threading
import os
from pathlib import Path as FilePath
from flask import Flask, render_template, Response, send_from_directory, request
from flask_socketio import SocketIO
from auto_shepherd_sheep_localisation_ros2.utils.geo_converter import MapConverter
from auto_shepherd_sheep_localisation_ros2.boids_storage import BoidsStorage


class MapVisualiser(Node):
    def gps_cb(self, msg: NavSatFix):
        with self.lock:
            self.drone_gps = {
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "altitude": msg.altitude,
            }
        self._calculate_fov()
        self._send_update()

    def attitude_cb(self, msg: Vector3Stamped):
        with self.lock:
            self.drone_attitude = {
                "yaw": msg.vector.z,
                "pitch": msg.vector.y,
                "roll": msg.vector.x,
            }
        self._calculate_fov()
        self._send_update()

    def gimbal_cb(self, msg: Vector3Stamped):
        with self.lock:
            self.gimbal = {
                "yaw": msg.vector.x,  # Gimbal yaw is relative to drone
                "pitch": msg.vector.y,
                "roll": msg.vector.z,
            }
        self._calculate_fov()
        self._send_update()

    def sheep_paths_cb(self, msg: Path):
        with self.lock:
            self.sheep_positions.clear()
            for pose in msg.poses:
                sheep_id = pose.header.frame_id  # ID stored in frame_id
                lat = pose.pose.position.x
                lon = pose.pose.position.y
                self.sheep_positions[sheep_id] = {
                    "latitude": lat,
                    "longitude": lon,
                    "id": sheep_id,
                }
                # Append to history
                if sheep_id not in self.sheep_history:
                    self.sheep_history[sheep_id] = []
                self.sheep_history[sheep_id].append([lat, lon])
                # Cap history length
                if len(self.sheep_history[sheep_id]) > self.max_history_points:
                    self.sheep_history[sheep_id] = self.sheep_history[sheep_id][
                        -self.max_history_points :
                    ]
        self._send_update()

    def sheep_clusters_cb(self, msg: Path):
        clusters = []
        for pose in msg.poses:
            cluster_id = pose.header.frame_id or f"cluster_{len(clusters)+1}"
            lat = pose.pose.position.x
            lon = pose.pose.position.y
            size = pose.pose.position.z if pose.pose.position.z else 0.0
            clusters.append(
                {"id": cluster_id, "latitude": lat, "longitude": lon, "size": int(size)}
            )
        with self.lock:
            self.sheep_clusters = clusters
        self._send_update()

    def sheep_sim_cb(self, msg: Path):
        with self.lock:
            self.sheep_sim_positions.clear()
            for pose in msg.poses:
                sheep_id = pose.header.frame_id  # ID stored in frame_id
                lat = pose.pose.position.x
                lon = pose.pose.position.y
                self.sheep_sim_positions[sheep_id] = {
                    "latitude": lat,
                    "longitude": lon,
                    "id": sheep_id,
                }
        self._send_update()

    def _gps_fence_cb(self, msg: Path):
        """Initialize MapConverter when field boundary is received"""
        if self.map_converter is None and len(msg.poses) > 0:
            # Convert boundary points to list of (lat, lon) tuples
            boundary_coords = [
                (p.pose.position.x, p.pose.position.y) for p in msg.poses
            ]
            self.map_converter = MapConverter(boundary_coords)
            self.get_logger().info(f"MapConverter initialized with {len(boundary_coords)} boundary points")
            # Convert any pending goal now that converter is available
            if self.pending_sheep_goal is not None:
                try:
                    x_meters, y_meters = self.pending_sheep_goal
                    lat, lon = self.map_converter.xy_to_latlon(y_meters, x_meters)
                    with self.lock:
                        self.sheep_goal = {"latitude": lat, "longitude": lon}
                        self.pending_sheep_goal = None
                    self._send_update()
                    self.get_logger().info("Converted pending sheep goal after MapConverter init")
                except Exception as e:
                    self.get_logger().warn(f"Failed to convert pending goal after MapConverter init: {e}")

    def sheep_goal_cb(self, msg: PoseStamped):
        """Callback for sheep herding goal position (XY in meters)"""
        # Extract XY coordinates (meters)
        x_meters = msg.pose.position.x
        y_meters = msg.pose.position.y

        # If converter is not ready, store pending goal and wait for boundary
        if self.map_converter is None:
            with self.lock:
                self.pending_sheep_goal = (x_meters, y_meters)
            self.get_logger().warn("sheep_goal_cb: MapConverter not ready — storing pending goal until boundary received")
            return

        # Convert XY coordinates to GPS
        try:
            lat, lon = self.map_converter.xy_to_latlon(y_meters, x_meters)
            with self.lock:
                self.sheep_goal = {"latitude": lat, "longitude": lon}
            self._send_update()
        except Exception as e:
            self.get_logger().warn(f"Failed to convert goal XY to GPS: {e}")

    def dog_command_cb(self, msg: PoseStamped):
        """Callback for dog/drone command positions (XY in meters)"""
        if self.map_converter is None:
            self.get_logger().warn("dog_command_cb: map_converter not initialized yet (waiting for field boundary)")
            return  # Need field boundary first
        
        # Convert XY coordinates to GPS using map converter
        x_meters = msg.pose.position.x
        y_meters = msg.pose.position.y
        
        try:
            lat, lon = self.map_converter.xy_to_latlon(y_meters, x_meters)
            
            with self.lock:
                self.dog_position = {
                    "latitude": lat,
                    "longitude": lon
                }
                # Add to history trail (limit to last 100 points)
                self.dog_history.append([lat, lon])
                if len(self.dog_history) > 100:
                    self.dog_history.pop(0)
            
            self._send_update()
        except Exception as e:
            self.get_logger().warn(f"Failed to convert dog XY to GPS: {e}")
    def video_cb(self, msg: Image):
        """Store latest detection frame"""
        try:
            with self.lock:
                self.latest_frame = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding="bgr8"
                )
        except Exception as e:
            self.get_logger().warn(f"Failed to convert video frame: {e}")

    def replay_status_cb(self, msg: String):
        """Bridge replay timing metadata to the dashboard."""
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict):
                raise ValueError("replay status is not an object")
            with self.lock:
                self.replay_status = status
            self._send_update()
        except Exception as exc:
            self.get_logger().warn(f"Failed to consume replay status: {exc}")

    def boids_analysis_cb(self, msg: String):
        """Bridge versioned Boids results to Socket.IO and durable history."""
        try:
            result = json.loads(msg.data)
            if not isinstance(result, dict) or result.get("schema_version") != 1:
                raise ValueError("unsupported Boids result schema")
            if self.boids_storage is not None and not self.boids_storage.save(result):
                return
            with self.lock:
                self.boids_analysis = result
                self.boids_history.append(result)
                if len(self.boids_history) > self.max_boids_history:
                    self.boids_history = self.boids_history[-self.max_boids_history :]
            self._send_update()
            self._send_boids_history()
        except Exception as exc:
            self.get_logger().warn(f"Failed to consume Boids analysis: {exc}")

    def replay_reset_cb(self, _msg: Empty):
        """Clear live trails/markers when replay starts a new source segment."""
        with self.lock:
            self.sheep_positions.clear()
            self.sheep_history.clear()
            self.sheep_clusters = []
            self.boids_analysis = None
            self.boids_history = []
            self.replay_reset_serial += 1
        self._send_update()
        self._send_boids_history()

    def _send_boids_history(self):
        try:
            with self.lock:
                history = list(self.boids_history)
                latest = self.boids_analysis
            self.socketio.emit("boids_analysis_update", latest, to=None)
            self.socketio.emit("boids_analysis_history", history, to=None)
        except Exception as exc:
            self.get_logger().warn(f"Failed to send Boids update: {exc}")

    def _generate_frames(self):
        """Generator for MJPEG video stream"""
        while True:
            with self.lock:
                if self.latest_frame is not None:
                    # Encode frame as JPEG
                    ret, buffer = cv2.imencode(
                        ".jpg", self.latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 85]
                    )
                    if ret:
                        frame = buffer.tobytes()
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                        )
            # Small delay to avoid consuming too much CPU
            import time

            time.sleep(0.033)  # ~30 FPS

    def _calculate_fov(self):
        """Calculate the 4 corners of the camera FOV on the ground"""
        with self.lock:
            if (
                self.drone_gps is None
                or self.gimbal is None
                or self.drone_attitude is None
            ):
                return

            altitude = self.drone_gps["altitude"]
            lat = self.drone_gps["latitude"]
            lon = self.drone_gps["longitude"]
            gimbal_pitch = self.gimbal["pitch"]
            # Absolute world yaw = drone yaw + gimbal yaw (relative to drone)
            absolute_yaw = self.drone_attitude["yaw"] + self.gimbal["yaw"]

            # Calculate the 4 corner pixels
            corners_px = [
                (0, 0),  # Top-left
                (self.image_width, 0),  # Top-right
                (self.image_width, self.image_height),  # Bottom-right
                (0, self.image_height),  # Bottom-left
            ]

            corner_coords = []
            for px, py in corners_px:
                # Get GPS for this corner
                corner_lat, corner_lon = self._pixel_to_gps(
                    px, py, lat, lon, altitude, gimbal_pitch, absolute_yaw
                )
                if corner_lat is not None:
                    corner_coords.append([corner_lat, corner_lon])

            self.camera_fov_corners = corner_coords

    def _pixel_to_gps(
        self, px, py, drone_lat, drone_lon, altitude, gimbal_pitch, gimbal_yaw
    ):
        """Convert pixel to GPS (same as in sheepdetectROS.py)"""
        try:
            # Normalize pixel coordinates to [-1, 1]
            norm_x = (px / self.image_width) * 2 - 1
            norm_y = (py / self.image_height) * 2 - 1

            # Convert to sensor coordinates
            sensor_x = norm_x * (self.sensor_width / 2)
            sensor_y = norm_y * (self.sensor_height / 2)

            # Create ray in camera frame (z forward, x right, y down)
            ray_x = sensor_x
            ray_y = sensor_y
            ray_z = self.focal_length

            # Normalize
            length = math.sqrt(ray_x**2 + ray_y**2 + ray_z**2)
            ray_x /= length
            ray_y /= length
            ray_z /= length

            # Apply pitch rotation (around X-axis)
            pitch_rad = math.radians(gimbal_pitch)
            cos_p = math.cos(pitch_rad)
            sin_p = math.sin(pitch_rad)
            ray_y_rot = ray_y * cos_p + ray_z * sin_p
            ray_z_rot = -ray_y * sin_p + ray_z * cos_p

            # Apply yaw rotation to convert to world frame
            # Gimbal yaw: 0=North, 90=East, 180/-180=South, -90=West
            yaw_rad = math.radians(gimbal_yaw)
            cos_y = math.cos(yaw_rad)
            sin_y = math.sin(yaw_rad)

            # Convert to world frame (x=East, y=North, z=Up)
            ray_north = ray_z_rot * cos_y - ray_x * sin_y
            ray_east = ray_z_rot * sin_y + ray_x * cos_y
            ray_up = ray_y_rot

            # Ray-ground intersection
            if ray_up >= 0:
                return None, None

            t = -altitude / ray_up
            ground_offset_north = ray_north * t
            ground_offset_east = ray_east * t

            # Convert to GPS
            lat_per_meter = 1.0 / 111320.0
            lon_per_meter = 1.0 / (
                40008000.0 * math.cos(math.radians(drone_lat)) / 360.0
            )

            new_lat = drone_lat + (ground_offset_north * lat_per_meter)
            new_lon = drone_lon + (ground_offset_east * lon_per_meter)

            return new_lat, new_lon
        except:
            return None, None

    def _send_update(self):
        """Send current state to all connected web clients"""
        try:
            gpu_status = self._read_gpu_status()
            with self.lock:
                data = {
                    "drone": self.drone_gps,
                    "sheep": list(self.sheep_positions.values()),
                    "sheep_sim": list(self.sheep_sim_positions.values()),
                    "sheep_clusters": self.sheep_clusters,
                    "sheep_goal": self.sheep_goal,
                    "camera_fov": self.camera_fov_corners,
                    "sheep_paths": self.sheep_history,
                    "dog_position": self.dog_position,
                    "dog_trail": self.dog_history,
                    "boids_analysis": self.boids_analysis,
                    "replay": self.replay_status,
                    "replay_reset_serial": self.replay_reset_serial,
                    "inference_size": self.inference_size,
                    "inference_mode": self.inference_mode,
                    "gpu": gpu_status,
                }
            self.socketio.emit("map_update", data, to=None)
        except Exception as e:
            self.get_logger().warn(f"Failed to send update: {e}")

    def _read_gpu_status(self):
        """Read NVIDIA telemetry through NVML at most once per second."""
        now = time.monotonic()
        if now - self._gpu_status_read_at < 1.0:
            return self._gpu_status_cache
        self._gpu_status_read_at = now

        if pynvml is None:
            self._gpu_status_cache = {
                "available": False,
                "reason": "nvidia-ml-py is not installed",
            }
            return self._gpu_status_cache

        try:
            if not self._nvml_initialized:
                pynvml.nvmlInit()
                self._nvml_initialized = True

            devices = []
            for index in range(pynvml.nvmlDeviceGetCount()):
                handle = pynvml.nvmlDeviceGetHandleByIndex(index)
                name = pynvml.nvmlDeviceGetName(handle)
                if isinstance(name, bytes):
                    name = name.decode("utf-8", errors="replace")
                utilisation = pynvml.nvmlDeviceGetUtilizationRates(handle)
                memory = pynvml.nvmlDeviceGetMemoryInfo(handle)
                devices.append({
                    "index": str(index),
                    "name": name,
                    "utilisation_percent": float(utilisation.gpu),
                    "memory_used_mib": float(memory.used) / (1024 * 1024),
                    "memory_total_mib": float(memory.total) / (1024 * 1024),
                    "temperature_c": float(
                        pynvml.nvmlDeviceGetTemperature(handle, pynvml.NVML_TEMPERATURE_GPU)
                    ),
                })
            self._gpu_status_cache = {
                "available": bool(devices),
                "devices": devices,
                "timestamp": time.time(),
            }
        except Exception as exc:
            if isinstance(exc, getattr(pynvml, "NVMLError", ())):
                self._nvml_initialized = False
            self._gpu_status_cache = {"available": False, "reason": str(exc)}
        return self._gpu_status_cache

    def _publish_boundary(self, coords):
        """Publish field boundary as Path message on /field/gps_fence/path"""
        try:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"
            
            for lat, lon in coords:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = lat
                pose.pose.position.y = lon
                pose.pose.position.z = 0.0
                path_msg.poses.append(pose)
            
            self.boundary_pub.publish(path_msg)
            self.get_logger().info(f"Published boundary with {len(coords)} points to /field/gps_fence/path")
        except Exception as e:
            self.get_logger().error(f"Failed to publish boundary: {e}")

    def _discover_replays(self, package_dir):
        """Return safe dashboard choices for MP4 files with matching SRT files."""
        models_dir = FilePath(package_dir) / "detection_process" / "models"
        roots = [models_dir / "videos", models_dir / "samples"]
        choices = []
        for root in roots:
            if not root.is_dir():
                continue
            for video_path in sorted(root.rglob("*")):
                if not video_path.is_file() or video_path.suffix.lower() != ".mp4":
                    continue
                srt_candidates = [
                    video_path.with_suffix(".srt"),
                    video_path.with_suffix(".SRT"),
                ]
                srt_path = next((path for path in srt_candidates if path.is_file()), None)
                if srt_path is None:
                    continue
                relative_video = video_path.relative_to(models_dir).as_posix()
                choices.append({
                    "id": relative_video,
                    "label": relative_video,
                    "duration_s": self._replay_duration_seconds(video_path, srt_path),
                    "video_path": str(video_path),
                    "srt_path": str(srt_path),
                })
        return choices

    @staticmethod
    def _replay_duration_seconds(video_path, srt_path):
        """Estimate replay length using video frames and SRT timing."""
        duration_s = 0.0
        capture = cv2.VideoCapture(str(video_path))
        try:
            fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
            frames = float(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0.0)
            if fps > 0.0 and frames > 0.0:
                duration_s = frames / fps
        finally:
            capture.release()

        try:
            text = FilePath(srt_path).read_text(errors="ignore")
            times = re.findall(
                r"(\d{2}:\d{2}:\d{2}[,.]\d{3})\s*-->\s*(\d{2}:\d{2}:\d{2}[,.]\d{3})",
                text,
            )
            if times:
                def seconds(value):
                    hours, minutes, rest = value.replace(",", ".").split(":")
                    return int(hours) * 3600 + int(minutes) * 60 + float(rest)
                duration_s = max(duration_s, seconds(times[-1][1]) - seconds(times[0][0]))
        except (OSError, ValueError):
            pass
        return round(max(0.0, duration_s), 3)

    @staticmethod
    def _discover_models(package_dir):
        """Return safe model choices from the packaged sample-model directory."""
        model_dir = FilePath(package_dir) / "detection_process" / "models" / "samples" / "sample_model"
        configured_name = FilePath(os.environ.get("YOLO_WEIGHTS", "")).name
        default_name = configured_name or "Aerial-Auth-Asfenah-DeepBack-PrabsUoL3-9.pt"
        choices = []
        model_paths = sorted(model_dir.iterdir()) if model_dir.is_dir() else []
        for model_path in model_paths:
            if not model_path.is_file() or model_path.suffix.lower() not in {".pt", ".onnx", ".engine"}:
                continue
            model_id = model_path.name
            choices.append({
                "id": model_id,
                "label": model_id,
                "default": model_id == default_name,
            })
        if choices and not any(item["default"] for item in choices):
            choices[0]["default"] = True
        return choices

    def __init__(self):
        super().__init__("map_visualiser")

        self.get_logger().info("🗺️ Map Visualiser Node Starting...")

        # Store latest data
        self.drone_gps = None
        self.sheep_positions = {}  # {sheep_id: (lat, lon)}
        self.sheep_history = {}  # {sheep_id: [[lat, lon], ...]}
        self.sheep_sim_positions = {}  # {sheep_id: (lat, lon)} - simulated sheep
        self.sheep_goal = None  # {'latitude': float, 'longitude': float} - herding goal
        self.dog_position = None  # {'latitude': float, 'longitude': float}
        self.dog_history = []  # [[lat, lon], ...] - dog path trail
        self.map_converter = None  # Initialized when field boundary is received
        self.sheep_clusters = (
            []
        )  # [{'id': str, 'latitude': float, 'longitude': float, 'size': int}]
        self.latest_frame = None
        self.replay_status = None
        self.replay_reset_serial = 0
        try:
            self.inference_size = int(os.getenv("YOLO_IMGSZ", "640"))
        except (TypeError, ValueError):
            self.inference_size = 640
        if self.inference_size not in {640, 960, 1280}:
            self.inference_size = 640
        self.inference_mode = os.getenv("YOLO_TILING", "full").strip().lower()
        if self.inference_mode not in {"full", "2x2"}:
            self.inference_mode = "full"
        self._gpu_status_cache = {"available": False, "reason": "waiting for first reading"}
        self._gpu_status_read_at = 0.0
        self._nvml_initialized = False
        self.boids_analysis = None
        self.max_boids_history = 240
        try:
            self.boids_storage = BoidsStorage()
            self.boids_history = self.boids_storage.history(limit=self.max_boids_history)
            self.boids_analysis = self.boids_history[-1] if self.boids_history else None
        except Exception as exc:
            self.get_logger().warn(f"Boids history storage unavailable: {exc}")
            self.boids_storage = None
            self.boids_history = []
        self.gimbal = None
        self.drone_attitude = None
        self.camera_fov_corners = []
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.max_history_points = 120  # limit trail length
        self.pending_sheep_goal = None  # store goal XY until map_converter available

        # Camera specs (Zenmuse H20)
        self.focal_length = 4.5  # mm
        self.sensor_width = 5.4 # 6.17  # mm
        self.sensor_height = 3.0 # 3.47 # 4.55  # mm
        self.image_width = 1920
        self.image_height = 1080

        # Subscribe to drone GPS and sheep paths
        self.create_subscription(NavSatFix, "/drone/gps", self.gps_cb, 10)
        self.create_subscription(Path, "/sheep_paths", self.sheep_paths_cb, 10)
        self.create_subscription(Path, "/sheep/poses_sim", self.sheep_sim_cb, 10)
        self.create_subscription(Path, "/sheep_clusters", self.sheep_clusters_cb, 10)
        self.create_subscription(PoseStamped, "/dog/command", self.dog_command_cb, 10)
        self.create_subscription(PoseStamped, "/sheep/goal", self.sheep_goal_cb, 10)
        self.create_subscription(Image, "/sheep_detections", self.video_cb, 10)
        self.create_subscription(String, "/drone/replay_status", self.replay_status_cb, 10)
        self.create_subscription(String, "/sheep/boids_analysis", self.boids_analysis_cb, 10)
        self.create_subscription(Empty, "/drone/replay_reset", self.replay_reset_cb, 10)
        self.create_subscription(Vector3Stamped, "/drone/gimbal", self.gimbal_cb, 10)
        self.create_subscription(
            Vector3Stamped, "/drone/attitude", self.attitude_cb, 10
        )

        # Publisher for field boundary
        boundary_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.boundary_pub = self.create_publisher(Path, "/field/gps_fence/path", boundary_qos)
        self.replay_select_pub = self.create_publisher(String, "/drone/select_replay", 10)
        self.model_select_pub = self.create_publisher(String, "/drone/select_model", 10)
        self.inference_size_pub = self.create_publisher(String, "/drone/select_inference_size", 10)
        self.replay_command_pub = self.create_publisher(String, "/drone/replay_command", 10)

        # Setup Flask and SocketIO
        pkg_dir = os.path.dirname(__file__)
        self.replay_catalog = self._discover_replays(pkg_dir)
        self.model_catalog = self._discover_models(pkg_dir)
        self.get_logger().info(
            f"Discovered {len(self.replay_catalog)} replay(s) with matching MP4/SRT files"
        )
        self.get_logger().info(f"Discovered {len(self.model_catalog)} detection model(s)")
        self.app = Flask(
            __name__,
            template_folder=os.path.join(pkg_dir, "web_templates"),
            static_folder=os.path.join(pkg_dir, "web_static"),
        )
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")

        # Setup routes
        @self.app.route("/")
        def index():
            return render_template("map.html")

        @self.app.route("/analysis")
        def analysis():
            return render_template("analysis.html")

        @self.app.route("/static/<path:filename>")
        def serve_static(filename):
            return send_from_directory(os.path.join(pkg_dir, "web_static"), filename)

        @self.app.route("/video_feed")
        def video_feed():
            return Response(
                self._generate_frames(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @self.app.route("/replays")
        def replays():
            # Do not expose filesystem paths to the browser.
            self.replay_catalog = self._discover_replays(pkg_dir)
            return {
                "replays": [
                    {"id": item["id"], "label": item["label"], "duration_s": item["duration_s"]}
                    for item in self.replay_catalog
                ]
            }

        @self.app.route("/models")
        def models():
            self.model_catalog = self._discover_models(pkg_dir)
            return {"models": self.model_catalog}

        @self.app.route("/inference_sizes")
        def inference_sizes():
            return {
                "sizes": [
                    {"value": 640, "label": "640 (fastest)"},
                    {"value": 960, "label": "960 (recommended test)"},
                    {"value": 1280, "label": "1280 (slowest / may use more memory)"},
                ],
                "selected": self.inference_size,
                "modes": [
                    {"value": "full", "label": "Full frame (standard)"},
                    {"value": "2x2", "label": "2×2 tiled (experimental)"},
                ],
                "selected_mode": self.inference_mode,
            }

        def selected_boids_history():
            try:
                limit = max(1, min(int(request.args.get("limit", 2000)), 2000))
            except (TypeError, ValueError):
                limit = 2000
            session_id = request.args.get("session_id") or None
            field_id = request.args.get("field_id") or None
            segment_id = request.args.get("segment_id") or None
            source_type = request.args.get("source_type") or None
            if self.boids_storage:
                values = self.boids_storage.history(
                    limit=limit,
                    session_id=session_id,
                    field_id=field_id,
                    segment_id=segment_id,
                )
            else:
                values = list(self.boids_history)
                values = [
                    value for value in values
                    if (not session_id or value.get("session_id") == session_id)
                    and (not field_id or value.get("field_id") == field_id)
                    and (not segment_id or value.get("segment_id") == segment_id)
                ][-limit:]
            if source_type:
                values = [value for value in values if value.get("source_type") == source_type]
            return values

        @self.app.route("/boids/history")
        def boids_history():
            try:
                return {"history": selected_boids_history()}
            except Exception as exc:
                self.get_logger().warn(f"Boids history request failed: {exc}")
                return {"history": [], "error": "history_unavailable"}, 503

        @self.app.route("/boids/filters")
        def boids_filters():
            try:
                values = selected_boids_history()
                return {
                    "sessions": sorted({value.get("session_id") for value in values if value.get("session_id")}),
                    "fields": sorted({value.get("field_id") for value in values if value.get("field_id")}),
                    "segments": sorted({value.get("segment_id") for value in values if value.get("segment_id")}),
                    "source_types": sorted({value.get("source_type") for value in values if value.get("source_type")}),
                }
            except Exception as exc:
                self.get_logger().warn(f"Boids filter request failed: {exc}")
                return {"sessions": [], "fields": [], "segments": [], "source_types": []}, 503

        @self.app.route("/boids/sessions")
        def boids_sessions():
            """Return session summaries for the research page."""
            try:
                summaries = {}
                for value in selected_boids_history():
                    session_id = value.get("session_id")
                    if not session_id:
                        continue
                    summary = summaries.setdefault(session_id, {
                        "session_id": session_id,
                        "source_type": value.get("source_type"),
                        "field_id": value.get("field_id"),
                        "replay_id": value.get("replay_id"),
                        "result_count": 0,
                        "last_window_end_s": None,
                    })
                    summary["result_count"] += 1
                    summary["last_window_end_s"] = value.get("window_end_s")
                return {"sessions": list(summaries.values())}
            except Exception as exc:
                self.get_logger().warn(f"Boids session request failed: {exc}")
                return {"sessions": []}, 503

        @self.app.route("/boids/session/delete", methods=["POST"])
        def delete_boids_session():
            """Delete one confirmed session; never delete source video files."""
            payload = request.get_json(silent=True) or {}
            session_id = str(payload.get("session_id", "")).strip()
            if payload.get("confirm") is not True:
                return {"ok": False, "error": "explicit confirmation required"}, 400
            if not session_id or len(session_id) > 512:
                return {"ok": False, "error": "invalid session_id"}, 400
            try:
                deleted = self.boids_storage.delete_session(session_id) if self.boids_storage else 0
                with self.lock:
                    before = len(self.boids_history)
                    self.boids_history = [
                        value for value in self.boids_history
                        if value.get("session_id") != session_id
                    ]
                    deleted += before - len(self.boids_history)
                    if self.boids_analysis and self.boids_analysis.get("session_id") == session_id:
                        self.boids_analysis = None
                self._send_update()
                self._send_boids_history()
                return {"ok": True, "session_id": session_id, "deleted_results": deleted}
            except Exception as exc:
                self.get_logger().warn(f"Boids session deletion failed: {exc}")
                return {"ok": False, "error": "session deletion failed"}, 500

        @self.app.route("/boids/export.csv")
        def boids_export():
            try:
                values = selected_boids_history()
                from flask import Response as FlaskResponse
                return FlaskResponse(
                    self.boids_storage.export_csv(values) if self.boids_storage else "",
                    mimetype="text/csv",
                    headers={"Content-Disposition": "attachment; filename=boids_analysis.csv"},
                )
            except Exception as exc:
                self.get_logger().warn(f"Boids export failed: {exc}")
                return {"error": "history_unavailable"}, 503

        @self.app.route("/boids/export_tracks.csv")
        def boids_track_export():
            try:
                values = selected_boids_history()
                from flask import Response as FlaskResponse
                return FlaskResponse(
                    self.boids_storage.export_track_csv(values) if self.boids_storage else "",
                    mimetype="text/csv",
                    headers={"Content-Disposition": "attachment; filename=boids_track_screening.csv"},
                )
            except Exception as exc:
                self.get_logger().warn(f"Per-track Boids export failed: {exc}")
                return {"error": "history_unavailable"}, 503

        @self.app.route("/boids/export.zip")
        def boids_bundle_export():
            try:
                import io
                import zipfile
                values = selected_boids_history()
                archive = io.BytesIO()
                with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as bundle:
                    flock_csv = self.boids_storage.export_csv(values) if self.boids_storage else ""
                    track_csv = self.boids_storage.export_track_csv(values) if self.boids_storage else ""
                    bundle.writestr("boids_flock_results.csv", flock_csv)
                    bundle.writestr("boids_track_screening.csv", track_csv)
                from flask import Response as FlaskResponse
                return FlaskResponse(
                    archive.getvalue(),
                    mimetype="application/zip",
                    headers={"Content-Disposition": "attachment; filename=boids_analysis_bundle.zip"},
                )
            except Exception as exc:
                self.get_logger().warn(f"Boids bundle export failed: {exc}")
                return {"error": "history_unavailable"}, 503

        @self.socketio.on("connect")
        def handle_connect():
            self.get_logger().info("Web client connected")
            self._send_update()
            self._send_boids_history()

        @self.socketio.on("field_boundary")
        def handle_boundary(coords):
            self.get_logger().info(f"Received field boundary with {len(coords)} points")
            # Publish boundary to ROS topic
            self._publish_boundary(coords)
            # Initialize MapConverter so we can convert local XY (meters) to GPS
            try:
                self.map_converter = MapConverter(coords)
                self.get_logger().info("MapConverter initialized from web-drawn field boundary")
            except Exception as e:
                self.get_logger().warn(f"Failed to initialize MapConverter from boundary: {e}")

        @self.socketio.on("clear_boundary")
        def handle_clear_boundary():
            self.get_logger().info("Clearing field boundary")
            # Publish empty boundary
            self._publish_boundary([])
            # Clear MapConverter so conversions stop
            self.map_converter = None
            self.get_logger().info("MapConverter cleared")

        @self.socketio.on("select_replay")
        def handle_select_replay(selection):
            """Send validated replay and detector-model choices to ROS."""
            self.replay_catalog = self._discover_replays(pkg_dir)
            self.model_catalog = self._discover_models(pkg_dir)
            selection = selection if isinstance(selection, dict) else {}
            replay_id = str(selection.get("id", "")).strip()
            replay = next((item for item in self.replay_catalog if item["id"] == replay_id), None)
            if replay is None:
                return {"ok": False, "error": "Replay is unavailable or has no matching SRT file"}
            model_id = str(selection.get("model_id", "")).strip()
            model = next((item for item in self.model_catalog if item["id"] == model_id), None)
            if model is None:
                model = next((item for item in self.model_catalog if item.get("default")), None)
            if model is None:
                return {"ok": False, "error": "No detection model is available"}

            model_message = String()
            model_message.data = json.dumps({"model_id": model["id"]})
            self.model_select_pub.publish(model_message)

            message = String()
            message.data = json.dumps({
                "replay_id": replay["id"],
                "video_path": replay["video_path"],
                "srt_path": replay["srt_path"],
                "model_id": model["id"],
            })
            self.replay_select_pub.publish(message)
            self.get_logger().info(
                f"Dashboard selected replay: {replay['label']} with model {model['label']}"
            )
            return {"ok": True, "label": replay["label"], "model_label": model["label"]}

        @self.socketio.on("select_inference_size")
        def handle_select_inference_size(selection):
            """Forward validated size and tiling settings to the detector."""
            selection = selection if isinstance(selection, dict) else {}
            try:
                size = int(selection.get("size", selection.get("imgsz")))
            except (TypeError, ValueError):
                return {"ok": False, "error": "Inference size must be 640, 960, or 1280"}
            if size not in {640, 960, 1280}:
                return {"ok": False, "error": "Inference size must be 640, 960, or 1280"}
            mode = str(selection.get("mode", selection.get("tiling_mode", self.inference_mode))).strip().lower()
            if mode not in {"full", "2x2"}:
                return {"ok": False, "error": "Inference mode must be full or 2x2"}
            message = String()
            message.data = json.dumps({"imgsz": size, "mode": mode})
            self.inference_size_pub.publish(message)
            self.inference_size = size
            self.inference_mode = mode
            self._send_update()
            self.get_logger().info(f"Dashboard selected YOLO inference settings: size={size}, mode={mode}")
            return {"ok": True, "size": size, "mode": mode}

        @self.socketio.on("replay_control")
        def handle_replay_control(command_payload):
            """Forward dashboard playback and loop commands to the loader."""
            command = str(command_payload.get("command", "")).lower() if isinstance(command_payload, dict) else ""
            if command not in {"play", "pause", "reset", "loop"}:
                return {"ok": False, "error": "Unknown replay command"}
            message = String()
            payload = {"command": command}
            if command == "loop":
                payload["enabled"] = bool(command_payload.get("enabled", False))
            message.data = json.dumps(payload)
            self.replay_command_pub.publish(message)
            self.get_logger().info(f"Dashboard replay command: {command}{'=' + str(payload['enabled']) if command == 'loop' else ''}")
            return {"ok": True, "command": command, "enabled": payload.get("enabled")}

        # Start Flask in background thread
        self.flask_thread = threading.Thread(
            target=lambda: self.socketio.run(
                self.app,
                host="0.0.0.0",
                port=8080,
                debug=True,
                use_reloader=False,
                allow_unsafe_werkzeug=True,
            )
        )
        self.flask_thread.daemon = True
        self.flask_thread.start()

        self.get_logger().info("✅ Map Visualiser Ready - Open http://localhost:8080")


def main(args=None):
    rclpy.init(args=args)
    node = MapVisualiser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Allow graceful shutdown on Ctrl+C without printing a stack trace
        pass
    finally:
        if getattr(node, "boids_storage", None) is not None:
            node.boids_storage.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
