#!/usr/bin/env python3

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import Image, NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from auto_shepherd_sheep_localisation_ros2.detection_process.modules.sheepdetectROS import SheepDetectROS
from auto_shepherd_sheep_localisation_ros2.detection_process.modules.config import *
from ultralytics import YOLO


class SheepDetector(Node):
    def __init__(self):
        super().__init__("sheep_detector")
        print('\n\n\n')

        # latest GPS fix (updated by gps_cb)
        self.gps = None

        # publisher for the outgoing Path
        self.path_pub = self.create_publisher(Path, "/sheep_paths", 10)

        # subscribe to latched GPS and live image topics
        self.create_subscription(NavSatFix, "/gps", self.gps_cb, self.qos())
        self.create_subscription(Image, "/drone_feed", self.image_cb, 10)

        # Odentify weights filepath
        YOLO_WEIGHTS_SHEEP = os.getenv('YOLO_WEIGHTS')
        YOLO_TRACKER = os.getenv('YOLO_TRACKER')

        self.SD = SheepDetectROS(YOLO_WEIGHTS_SHEEP, YOLO_TRACKER,
            conf=SC,
            iou=SIOU,
            agnostic_nms=SA,
            max_det=SM,
            verbose=SV,
            stream=SS,
            )
        self.bridge = CvBridge()
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
    def image_cb(self, msg: Image):
        if self.gps is None:
            return  # skip until we have GPS

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detections = self.SD.predict(frame, self.gps)

        path = Path()
        path.header.stamp = msg.header.stamp   # timestamp from the image
        path.header.frame_id = "map"

        # pack each (id, pose) pair into PoseStamped
        for i in range(len(detections[0])):
            sheep_id, pose = detections[0][i], detections[1][i]
            ps = PoseStamped()
            ps.header.stamp = msg.header.stamp
            ps.header.frame_id = str(sheep_id)     # use frame_id to store ID
            ps.pose.position.x = pose['position']['x']
            ps.pose.position.y = pose['position']['y']
            path.poses.append(psn)
        self.path_pub.publish(path)  # publish to /sheep_paths


def main():
    rclpy.init()
    node = SheepDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
