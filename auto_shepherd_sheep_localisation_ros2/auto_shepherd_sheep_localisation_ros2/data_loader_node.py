#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, String
from sensor_msgs.msg import NavSatFix, Image
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped
from cv_bridge import CvBridge
import cv2
import re
import time
import json
import threading
from pathlib import Path


class DroneDataPublisher(Node):
    def __init__(self):
        super().__init__("data_loader_node")

        # Parameters
        self.declare_parameter("video_path", "")
        self.declare_parameter("srt_path", "")
        self.declare_parameter("topic_name", "drone")
        self.declare_parameter("loop_video", False)

        # Get topic name
        topic = self.get_parameter("topic_name").value
        self.loop_video = bool(self.get_parameter("loop_video").value)

        # Publishers for individual data streams
        self.main_pub = self.create_publisher(String, topic, 10)
        self.gps_pub = self.create_publisher(NavSatFix, f"{topic}/gps", 10)
        self.speed_pub = self.create_publisher(Vector3Stamped, f"{topic}/speed", 10)
        self.attitude_pub = self.create_publisher(
            Vector3Stamped, f"{topic}/attitude", 10
        )
        self.gimbal_pub = self.create_publisher(Vector3Stamped, f"{topic}/gimbal", 10)
        self.camera_pub = self.create_publisher(String, f"{topic}/camera", 10)
        self.image_pub = self.create_publisher(Image, f"{topic}/image", 10)
        self.replay_reset_pub = self.create_publisher(Empty, "/drone/replay_reset", 10)
        self.replay_status_pub = self.create_publisher(String, "/drone/replay_status", 10)
        self.relative_altitude_pub = self.create_publisher(
            Float32, f"{topic}/relative_altitude", 10
        )
        self.absolute_altitude_pub = self.create_publisher(
            Float32, f"{topic}/absolute_altitude", 10
        )
        self.replay_selection_sub = self.create_subscription(
            String, "/drone/select_replay", self.replay_selection_cb, 10
        )
        self.replay_command_sub = self.create_subscription(
            String, "/drone/replay_command", self.replay_command_cb, 10
        )

        self.bridge = CvBridge()
        self._replay_lock = threading.Lock()
        self._requested_replay = None
        self._stop_playback = threading.Event()
        self._wake_playback = threading.Event()
        self._pause_playback = threading.Event()
        self._last_replay_status = None

        # Get file paths
        video_path = self.get_parameter("video_path").value
        srt_path = self.get_parameter("srt_path").value

        # Find files if not specified
        if not video_path or not srt_path:
            video_path, srt_path = self.find_sample_files()

        self.get_logger().info(f"Video: {video_path}")
        self.get_logger().info(f"SRT: {srt_path}")

        self._requested_replay = (str(video_path), str(srt_path))
        self.playback_thread = threading.Thread(
            target=self._playback_loop, name="replay-playback", daemon=True
        )
        self.playback_thread.start()

    def find_sample_files(self):
        """Search for sample video and SRT files"""
        search_paths = [
            Path(
                "/home/ros/base_ws/src/auto_shepherd_sheep_localisation_ros2/auto_shepherd_sheep_localisation_ros2/detection_process/samples"
            ),
            Path(
                "/home/jcox/code/auto_shepherd/auto_shepherd_sheep_localisation/auto_shepherd_sheep_localisation_ros2/auto_shepherd_sheep_localisation_ros2/detection_process/samples"
            ),
        ]

        for path in search_paths:
            if path.exists():
                videos = list(path.glob("*.MP4")) + list(path.glob("*.mp4"))
                if videos:
                    video = str(videos[0])
                    srt = str(videos[0].with_suffix(".SRT"))
                    if not Path(srt).exists():
                        srt = str(videos[0].with_suffix(".srt"))
                    return video, srt

        raise FileNotFoundError("Could not find sample video files")

    def replay_selection_cb(self, msg):
        """Switch playback to a dashboard-selected MP4/SRT pair."""
        try:
            selection = json.loads(msg.data)
            video_path = Path(str(selection["video_path"])).resolve()
            srt_path = Path(str(selection["srt_path"])).resolve()
        except (ValueError, KeyError, TypeError, OSError) as exc:
            self.get_logger().warning(f"Ignoring invalid replay selection: {exc}")
            return

        if video_path.suffix.lower() != ".mp4" or srt_path.suffix.lower() != ".srt":
            self.get_logger().warning("Ignoring replay selection with non-MP4/SRT files")
            return
        if not video_path.is_file() or not srt_path.is_file():
            self.get_logger().warning(
                f"Ignoring replay selection because files are unavailable: {video_path}, {srt_path}"
            )
            return

        with self._replay_lock:
            self._requested_replay = (str(video_path), str(srt_path))
        self._stop_playback.set()
        self._wake_playback.set()
        self.get_logger().info(f"Replay selection received: {video_path.name}")

    def replay_command_cb(self, msg):
        """Handle play, pause, and reset commands from the dashboard."""
        try:
            command = str(json.loads(msg.data).get("command", "")).lower()
        except (ValueError, TypeError, AttributeError):
            command = str(msg.data).lower().strip()

        if command == "pause":
            self._pause_playback.set()
            self._publish_current_replay_state("paused")
        elif command == "play":
            self._pause_playback.clear()
            self._wake_playback.set()
            self._publish_current_replay_state("playing")
        elif command == "reset":
            self._pause_playback.clear()
            self._stop_playback.set()
            self._wake_playback.set()
            self.get_logger().info("Replay reset requested: restarting current video and tracker state")
        else:
            self.get_logger().warning(f"Ignoring unknown replay command: {command}")

    def _playback_loop(self):
        """Run replay in a worker so ROS callbacks can change the source."""
        while rclpy.ok():
            with self._replay_lock:
                replay = self._requested_replay
            if replay is None:
                self._wake_playback.wait(timeout=1.0)
                self._wake_playback.clear()
                continue

            video_path, srt_path = replay
            self._stop_playback.clear()
            self._wake_playback.clear()
            self._pause_playback.clear()
            try:
                self.srt_data = self.parse_srt_file(srt_path)
                self.get_logger().info(
                    f"Selected replay: {video_path} ({len(self.srt_data)} SRT frames)"
                )
                self.replay_reset_pub.publish(Empty())
                self.stream_data(video_path)
            except Exception as exc:
                self.get_logger().error(f"Replay failed: {exc}")
                self._wake_playback.wait(timeout=1.0)
                self._wake_playback.clear()

            if not self._stop_playback.is_set() and not self.loop_video:
                break

    def stop_playback(self):
        """Stop the worker cleanly during ROS shutdown."""
        self._stop_playback.set()
        self._wake_playback.set()
        if getattr(self, "playback_thread", None) is not None:
            self.playback_thread.join(timeout=2.0)

    def publish_replay_status(self, video_path, current_time_s, duration_s, playing=True):
        """Publish timing metadata for the dashboard replay timeline."""
        status = {
            "video_name": Path(video_path).name,
            "start_time_s": 0.0,
            "current_time_s": max(0.0, float(current_time_s)),
            "end_time_s": max(0.0, float(duration_s)),
            "duration_s": max(0.0, float(duration_s)),
            "playing": bool(playing),
            "state": "playing" if playing else "ended",
        }
        with self._replay_lock:
            self._last_replay_status = status
        message = String()
        message.data = json.dumps(status)
        self.replay_status_pub.publish(message)

    def _publish_current_replay_state(self, state):
        """Publish a pause/play state without changing the current position."""
        with self._replay_lock:
            if self._last_replay_status is None:
                return
            status = dict(self._last_replay_status)
            status["playing"] = state == "playing"
            status["state"] = state
            self._last_replay_status = status
        message = String()
        message.data = json.dumps(status)
        self.replay_status_pub.publish(message)

    def parse_srt_file(self, srt_path):
        """Parse DJI SRT file and extract telemetry data"""
        with open(srt_path, "r") as f:
            content = f.read()

        # Split into subtitle blocks
        blocks = re.split(r"\n\n+", content.strip())
        data = []

        for block in blocks:
            lines = block.strip().split("\n")
            if len(lines) < 3:
                continue

            # Parse frame number
            try:
                frame_num = int(lines[0])
            except:
                continue

            # Parse timestamp
            time_match = re.match(
                r"(\d{2}:\d{2}:\d{2},\d{3})\s*-->\s*(\d{2}:\d{2}:\d{2},\d{3})", lines[1]
            )
            if not time_match:
                continue

            start_time = self.parse_timecode(time_match.group(1))

            # Parse telemetry data from remaining lines
            telemetry = {"frame": frame_num, "time": start_time}

            for line in lines[2:]:
                # Extract bracketed values like [iso: 100] or [drone_speedx: -0.1 drone_speedy: -0.2]
                matches = re.findall(r"\[([^\]]+)\]", line)
                for match in matches:
                    # Split by whitespace that precedes a key (word followed by colon)
                    # This handles both single entries and multiple entries in one bracket
                    key_value_pairs = re.findall(
                        r"(\w+)\s*:\s*([^\s:]+(?:\s+[^\s:]+)*?)(?=\s+\w+\s*:|$)", match
                    )
                    for key, value in key_value_pairs:
                        key = key.strip().replace(" ", "_")
                        value = value.strip()
                        telemetry[key] = self.convert_value(value)

            data.append(telemetry)

        return data

    def parse_timecode(self, tc):
        """Convert SRT timecode (HH:MM:SS,mmm) to seconds"""
        parts = tc.replace(",", ":").split(":")
        h, m, s, ms = map(int, parts)
        return h * 3600 + m * 60 + s + ms / 1000.0

    def convert_value(self, value):
        """Convert string value to appropriate type"""
        try:
            if "." in value:
                return float(value)
            return int(value)
        except:
            return value

    def stream_data(self, video_path):
        """Stream video and publish synchronized telemetry data"""
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            self.get_logger().error(f"Failed to open video: {video_path}")
            return

        fps = cap.get(cv2.CAP_PROP_FPS)
        frame_duration = 1.0 / fps if fps > 0 else 1.0 / 30.0
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        srt_start = float(self.srt_data[0].get("time", 0.0)) if self.srt_data else 0.0
        srt_end = float(self.srt_data[-1].get("time", srt_start)) if self.srt_data else srt_start
        duration_s = max(0.0, srt_end - srt_start, frame_count * frame_duration)

        self.get_logger().info(f"Streaming at {fps:.2f} FPS")
        self.publish_replay_status(video_path, 0.0, duration_s)

        frame_idx = 0
        start_time = time.time()
        replay_generation = 0

        while cap.isOpened() and rclpy.ok() and not self._stop_playback.is_set():
            while self._pause_playback.is_set() and not self._stop_playback.is_set() and rclpy.ok():
                self._wake_playback.wait(timeout=0.25)
                self._wake_playback.clear()
            if self._stop_playback.is_set() or not rclpy.ok():
                break
            ret, frame = cap.read()
            if not ret:
                if self.loop_video and not self._stop_playback.is_set():
                    # Loop: restart video and telemetry
                    replay_generation += 1
                    self.publish_replay_status(video_path, duration_s, duration_s, playing=False)
                    self.replay_reset_pub.publish(Empty())
                    self.get_logger().info(
                        f"Replay loop {replay_generation}: published /drone/replay_reset"
                    )
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    frame_idx = 0
                    start_time = time.time()
                    continue
                break

            # Get telemetry for this frame
            if frame_idx < len(self.srt_data):
                telemetry = self.srt_data[frame_idx]
                self.publish_telemetry(telemetry)
                current_time_s = float(telemetry.get("time", srt_start)) - srt_start
            else:
                current_time_s = frame_idx * frame_duration
            self.publish_replay_status(video_path, current_time_s, duration_s)

            # Publish image frame
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "drone_camera"
                self.image_pub.publish(img_msg)
            except Exception as exc:  # pragma: no cover
                self.get_logger().warn(f"Failed to publish image: {exc}")

            # Synchronize with video timing
            expected_time = start_time + (frame_idx * frame_duration)
            sleep_time = expected_time - time.time()
            if sleep_time > 0:
                self._wake_playback.wait(timeout=sleep_time)
                self._wake_playback.clear()

            frame_idx += 1

        cap.release()
        self.get_logger().info("Streaming stopped")

    def publish_telemetry(self, data):
        """Publish telemetry data to topics"""

        # Publish full JSON payload for easy echo
        try:
            payload = json.dumps(data)
            main_msg = String()
            main_msg.data = payload
            self.main_pub.publish(main_msg)
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f"Failed to publish JSON payload: {exc}")

        # GPS data
        if all(k in data for k in ["latitude", "longitude"]):
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = "drone"
            gps_msg.latitude = float(data.get("latitude", 0))
            gps_msg.longitude = float(data.get("longitude", 0))
            gps_msg.altitude = float(
                data.get("rel_alt", 0)
            )  # Use relative altitude for ground distance calculations
            self.gps_pub.publish(gps_msg)

        # Speed data
        if any(k in data for k in ["drone_speedx", "drone_speedy", "drone_speedz"]):
            speed_msg = Vector3Stamped()
            speed_msg.header.stamp = self.get_clock().now().to_msg()
            speed_msg.header.frame_id = "drone"
            speed_msg.vector.x = float(data.get("drone_speedx", 0))
            speed_msg.vector.y = float(data.get("drone_speedy", 0))
            speed_msg.vector.z = float(data.get("drone_speedz", 0))
            self.speed_pub.publish(speed_msg)

        # Attitude (yaw, pitch, roll)
        if any(k in data for k in ["drone_yaw", "drone_pitch", "drone_roll"]):
            att_msg = Vector3Stamped()
            att_msg.header.stamp = self.get_clock().now().to_msg()
            att_msg.header.frame_id = "drone"
            att_msg.vector.x = float(data.get("drone_yaw", 0))
            att_msg.vector.y = float(data.get("drone_pitch", 0))
            att_msg.vector.z = float(data.get("drone_roll", 0))
            self.attitude_pub.publish(att_msg)

        # Gimbal
        if any(k in data for k in ["gb_yaw", "gb_pitch", "gb_roll"]):
            gimbal_msg = Vector3Stamped()
            gimbal_msg.header.stamp = self.get_clock().now().to_msg()
            gimbal_msg.header.frame_id = "gimbal"
            gimbal_msg.vector.x = float(data.get("gb_yaw", 0))
            gimbal_msg.vector.y = float(data.get("gb_pitch", 0))
            gimbal_msg.vector.z = float(data.get("gb_roll", 0))
            self.gimbal_pub.publish(gimbal_msg)

        # Camera settings (as readable string)
        camera_info = []
        for key in ["iso", "shutter", "fnum", "ev", "focal_len", "dzoom"]:
            if key in data:
                camera_info.append(f"{key}: {data[key]}")

        if camera_info:
            camera_msg = String()
            camera_msg.data = " | ".join(camera_info)
            self.camera_pub.publish(camera_msg)

        # Relative altitude
        if "rel_alt" in data:
            rel_alt_msg = Float32()
            rel_alt_msg.data = float(data.get("rel_alt", 0))
            self.relative_altitude_pub.publish(rel_alt_msg)

        # Absolute altitude
        if "abs_alt" in data:
            abs_alt_msg = Float32()
            abs_alt_msg.data = float(data.get("abs_alt", 0))
            self.absolute_altitude_pub.publish(abs_alt_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_playback()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
