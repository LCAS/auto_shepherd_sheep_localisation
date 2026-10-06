#!/usr/bin/env python3

"""ROS2 adapter for the pure Boids estimator."""

from __future__ import annotations

import hashlib
import json
import math
import time
from typing import Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String
from std_msgs.msg import Empty

from auto_shepherd_sheep_localisation_ros2.boids_analysis import BoidsConfig, BoidsEstimator


class LocalMetricFrame:
    """Equirectangular local ENU frame adequate for a field-sized footprint."""

    def __init__(self, origin: Optional[Tuple[float, float]] = None):
        self.origin = origin

    def set_if_missing(self, points: Sequence[Tuple[float, float]]) -> None:
        if self.origin is None and points:
            self.origin = (
                sum(point[0] for point in points) / len(points),
                sum(point[1] for point in points) / len(points),
            )

    def to_xy(self, latitude: float, longitude: float) -> Optional[Tuple[float, float]]:
        if self.origin is None or not math.isfinite(latitude) or not math.isfinite(longitude):
            return None
        lat0, lon0 = self.origin
        north = (latitude - lat0) * 111320.0
        east = (longitude - lon0) * 111320.0 * math.cos(math.radians(lat0))
        return east, north


class BoidsAnalysisNode(Node):
    def __init__(self):
        super().__init__("boids_analysis")
        self.declare_parameter("source_type", "replay")
        self.declare_parameter("session_id", "session_001")
        self.declare_parameter("field_id", "field_default")
        self.declare_parameter("fit_window_s", 60.0)
        self.declare_parameter("publish_interval_s", 1.0)
        self.declare_parameter("minimum_window_s", 30.0)
        self.declare_parameter("minimum_samples", 60)
        self.declare_parameter("minimum_track_samples", 30)
        self.declare_parameter("maximum_gap_s", 2.0)
        self.declare_parameter("neighbour_radius_m", 15.0)
        self.declare_parameter("separation_radius_m", 5.0)
        self.declare_parameter("boundary_radius_m", 15.0)
        self.declare_parameter("regularisation", 0.01)
        self.declare_parameter("stale_after_s", 5.0)

        config = BoidsConfig(
            fit_window_s=float(self.get_parameter("fit_window_s").value),
            publish_interval_s=float(self.get_parameter("publish_interval_s").value),
            minimum_window_s=float(self.get_parameter("minimum_window_s").value),
            minimum_samples=int(self.get_parameter("minimum_samples").value),
            minimum_track_samples=int(self.get_parameter("minimum_track_samples").value),
            maximum_gap_s=float(self.get_parameter("maximum_gap_s").value),
            neighbour_radius_m=float(self.get_parameter("neighbour_radius_m").value),
            separation_radius_m=float(self.get_parameter("separation_radius_m").value),
            boundary_radius_m=float(self.get_parameter("boundary_radius_m").value),
            regularisation=float(self.get_parameter("regularisation").value),
            stale_after_s=float(self.get_parameter("stale_after_s").value),
        )
        self.estimator = BoidsEstimator(config)
        self.estimator.configure_identity(
            str(self.get_parameter("session_id").value),
            str(self.get_parameter("field_id").value),
            str(self.get_parameter("source_type").value),
        )
        self.frame = LocalMetricFrame()
        self.last_received_wall_time: Optional[float] = None
        self.last_stale_window: Optional[float] = None
        self.result_pub = self.create_publisher(String, "/sheep/boids_analysis", 10)
        boundary_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Path, "/sheep_paths", self._positions_cb, 10)
        self.create_subscription(Empty, "/drone/replay_reset", self._replay_reset_cb, 10)
        self.create_subscription(Path, "/field/gps_fence/path", self._boundary_cb, boundary_qos)
        self.create_timer(1.0, self._stale_cb)
        self.get_logger().info("Boids analysis node ready on /sheep/boids_analysis")

    @staticmethod
    def _stamp_seconds(path: Path) -> float:
        return float(path.header.stamp.sec) + float(path.header.stamp.nanosec) * 1.0e-9

    @staticmethod
    def _json_safe(value):
        if isinstance(value, dict):
            return {key: BoidsAnalysisNode._json_safe(item) for key, item in value.items()}
        if isinstance(value, list):
            return [BoidsAnalysisNode._json_safe(item) for item in value]
        if isinstance(value, float) and not math.isfinite(value):
            return None
        return value

    def _boundary_cb(self, msg: Path) -> None:
        gps_points = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in msg.poses
            if math.isfinite(float(pose.pose.position.x)) and math.isfinite(float(pose.pose.position.y))
        ]
        self.frame.set_if_missing(gps_points)
        local_points = [self.frame.to_xy(lat, lon) for lat, lon in gps_points]
        local_points = [point for point in local_points if point is not None]
        revision = hashlib.sha1(repr(gps_points).encode("utf-8")).hexdigest()[:12]
        self.estimator.set_boundary(local_points, revision=revision)

    def _replay_reset_cb(self, _msg: Empty) -> None:
        self.estimator.reset_segment("replay_reset")
        self.last_received_wall_time = None
        self.last_stale_window = None
        self.get_logger().info(
            f"Replay reset: started Boids segment {self.estimator.segment_id}"
        )

    def _positions_cb(self, msg: Path) -> None:
        gps_points = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in msg.poses
            if math.isfinite(float(pose.pose.position.x)) and math.isfinite(float(pose.pose.position.y))
        ]
        self.frame.set_if_missing(gps_points)
        positions = {}
        for pose in msg.poses:
            point = self.frame.to_xy(float(pose.pose.position.x), float(pose.pose.position.y))
            if point is not None:
                positions[pose.header.frame_id or "unknown"] = point
        timestamp = self._stamp_seconds(msg)
        if timestamp <= 0.0:
            timestamp = self.get_clock().now().nanoseconds * 1.0e-9
        if not self.estimator.ingest(timestamp, positions):
            return
        self.last_received_wall_time = time.monotonic()
        self.last_stale_window = None
        result = self.estimator.publish_result(wall_timestamp=self.get_clock().now().nanoseconds * 1.0e-9)
        if result is not None:
            result["model"]["coordinate_frame"] = {
                "type": "local_enu_equirectangular",
                "x": "east_m",
                "y": "north_m",
                "origin_latitude": self.frame.origin[0] if self.frame.origin else None,
                "origin_longitude": self.frame.origin[1] if self.frame.origin else None,
            }
            result["model"]["configuration_hash"] = hashlib.sha256(
                json.dumps(result["model"]["configuration"], sort_keys=True).encode("utf-8")
            ).hexdigest()[:16]
            self._publish(result)

    def _stale_cb(self) -> None:
        if self.last_received_wall_time is None or self.estimator._last_result is None:
            return
        if time.monotonic() - self.last_received_wall_time <= self.estimator.config.stale_after_s:
            return
        result = self.estimator._last_result
        if self.last_stale_window == result.get("window_end_s"):
            return
        stale = json.loads(json.dumps(result))
        stale["status"] = "stale"
        reasons = set(stale.setdefault("quality", {}).get("reason_codes", []))
        reasons.add("source_stale")
        stale["quality"]["reason_codes"] = sorted(reasons)
        stale["publication_time_s"] = self.get_clock().now().nanoseconds * 1.0e-9
        self.last_stale_window = result.get("window_end_s")
        self._publish(stale)

    def _publish(self, result: dict) -> None:
        message = String()
        message.data = json.dumps(self._json_safe(result), allow_nan=False)
        self.result_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = BoidsAnalysisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
