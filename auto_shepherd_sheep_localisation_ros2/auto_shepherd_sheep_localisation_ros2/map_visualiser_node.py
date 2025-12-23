#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, Image
from nav_msgs.msg import Path
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from cv_bridge import CvBridge
import cv2
import math

import threading
from flask import Flask, render_template, Response, send_from_directory
from flask_socketio import SocketIO
from auto_shepherd_sheep_localisation_ros2.utils.geo_converter import MapConverter


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
                }
            self.socketio.emit("map_update", data, to=None)
        except Exception as e:
            self.get_logger().warn(f"Failed to send update: {e}")

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

        # Setup Flask and SocketIO
        import os

        pkg_dir = os.path.dirname(__file__)
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

        @self.app.route("/static/<path:filename>")
        def serve_static(filename):
            return send_from_directory(os.path.join(pkg_dir, "web_static"), filename)

        @self.app.route("/video_feed")
        def video_feed():
            return Response(
                self._generate_frames(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @self.socketio.on("connect")
        def handle_connect():
            self.get_logger().info("Web client connected")
            self._send_update()

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

        # Start Flask in background thread
        self.flask_thread = threading.Thread(
            target=lambda: self.socketio.run(
                self.app, host="0.0.0.0", port=8080, debug=True, use_reloader=False
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
