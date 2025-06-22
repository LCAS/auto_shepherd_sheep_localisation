#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from sensor_msgs.msg import Image, NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from detection_process import detect_on_image    # <--- CODE GOES IN THIS FUNCTION TO ACTUALLY DETECT


class SheepDetector(Node):
    def __init__(self):
        super().__init__("sheep_detector")

        # latest GPS fix (updated by gps_cb)
        self.gps = None

        # publisher for the outgoing Path
        self.path_pub = self.create_publisher(Path, "/sheep_paths", 10)

        # subscribe to latched GPS and live image topics
        self.create_subscription(NavSatFix, "/gps", self.gps_cb, self.qos())
        self.create_subscription(Image, "/drone_feed", self.image_cb, 10)

    # convenience method for a latched QoS profile
    def qos(self):
        return QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

    # store the most-recent GPS fix
    def gps_cb(self, msg: NavSatFix):
        self.gps = msg

    # run detection on every frame and publish the result as a Path
    def image_cb(self, img_msg: Image):
        if self.gps is None:
            return  # skip until we have GPS

        detections = detect_on_image(img_msg, self.gps)

        path = Path()
        path.header.stamp = img_msg.header.stamp   # timestamp from the image
        path.header.frame_id = "map"

        # pack each (id, pose) pair into PoseStamped
        for sheep_id, pose in detections:
            ps = PoseStamped()
            ps.header.stamp = img_msg.header.stamp
            ps.header.frame_id = str(sheep_id)     # use frame_id to store ID
            ps.pose = pose
            path.poses.append(ps)

        self.path_pub.publish(path)  # publish to /sheep_paths


def main():
    rclpy.init()
    node = SheepDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
