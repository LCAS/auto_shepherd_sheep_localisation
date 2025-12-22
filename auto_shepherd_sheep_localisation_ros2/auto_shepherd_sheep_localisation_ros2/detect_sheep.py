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

        # publisher for the outgoing Path and Labelled image
        self.path_pub = self.create_publisher(Path, "/sheep/poses", 10)
        self.boxes_pub = self.create_publisher(Image, "/sheep/labelled_img", 10)

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

    def image_cb(self, msg: Image):
        if self.gps is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ids, poses, box_list = self.SD.predict(frame, self.gps)

        path = Path()
        path.header.stamp = msg.header.stamp
        path.header.frame_id = "map"

        for i in range(len(ids)):
            sheep_id = ids[i]
            pose = poses[i]
            boxes = box_list[i]  # this is a `Boxes` object (may have 1 or more boxes)

            # Draw all boxes for this detection
            for j in range(len(boxes.xyxy)):
                x1, y1, x2, y2 = map(int, boxes.xyxy[j].cpu().numpy())
                conf = float(boxes.conf[j]) if boxes.conf is not None else 0.0

                label = f"ID: {sheep_id} ({conf:.2f})"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Add pose to Path message
            ps = PoseStamped()
            ps.header.stamp = msg.header.stamp
            ps.header.frame_id = str(sheep_id)
            ps.pose.position.x = pose['position']['x']
            ps.pose.position.y = pose['position']['y']
            path.poses.append(ps)

        self.path_pub.publish(path)

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id
        self.boxes_pub.publish(img_msg)





def main():
    rclpy.init()
    node = SheepDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
