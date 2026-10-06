#!/usr/bin/env python3

"""Seeded synthetic stream for exercising the real Boids ROS/dashboard path."""

import math

import rclpy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile


class BoidsDemoNode(Node):
    def __init__(self):
        super().__init__("boids_demo")
        self.start = self.get_clock().now().nanoseconds * 1.0e-9
        self.boundary_pub = self.create_publisher(
            Path,
            "/field/gps_fence/path",
            QoSProfile(
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        self.positions_pub = self.create_publisher(Path, "/sheep_paths", 10)
        self.create_timer(0.1, self._publish)
        self._publish_boundary()
        self.get_logger().info("Synthetic Boids stream ready (source_type=simulation)")

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _publish_boundary(self):
        message = Path()
        message.header.stamp = self._stamp()
        message.header.frame_id = "field_demo"
        # Roughly 120 m x 80 m field around the seeded flock.
        for index, (east, north) in enumerate(
            [(-60.0, -40.0), (60.0, -40.0), (60.0, 40.0), (-60.0, 40.0)]
        ):
            pose = PoseStamped()
            pose.header.stamp = message.header.stamp
            pose.header.frame_id = f"boundary_{index}"
            pose.pose.position.x = 53.2660 + north / 111320.0
            pose.pose.position.y = -0.5290 + east / (111320.0 * math.cos(math.radians(53.2660)))
            message.poses.append(pose)
        self.boundary_pub.publish(message)

    def _publish(self):
        now = self.get_clock().now().nanoseconds * 1.0e-9
        t = now - self.start
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "simulation"
        # A slowly translating, breathing flock plus independent oscillations
        # creates non-zero, correlated and changing influence vectors.
        centre_east = 12.0 * math.sin(t / 12.0)
        centre_north = 9.0 * math.cos(t / 15.0)
        for index in range(10):
            angle = 2.0 * math.pi * index / 10.0
            radius = 14.0 + 3.0 * math.sin(t / 7.0 + index * 0.6)
            east = centre_east + radius * math.cos(angle) + 1.8 * math.sin(t * 0.7 + index)
            north = centre_north + radius * math.sin(angle) + 1.5 * math.cos(t * 0.5 + index * 0.4)
            pose = PoseStamped()
            pose.header.stamp = message.header.stamp
            pose.header.frame_id = f"sim_{index:02d}"
            pose.pose.position.x = 53.2660 + north / 111320.0
            pose.pose.position.y = -0.5290 + east / (111320.0 * math.cos(math.radians(53.2660)))
            message.poses.append(pose)
        self.positions_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = BoidsDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
