#!/usr/bin/env python3

# ROS 2 node: reads a video file path from /video_filepath,
# then streams each frame to /drone_feed and publishes a
# fixed GPS fix (53.01020 lat, –0.53434 lon, 45.24 m alt) on /gps.

import time
import threading
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import String
from sensor_msgs.msg import Image, NavSatFix


class VideoPublisher(Node):
    def __init__(self):
        super().__init__("video_publisher")

        # publishers ---------------------------------------------------------
        self.image_pub = self.create_publisher(Image, "/drone_feed", 10)
        self.gps_pub = self.create_publisher(NavSatFix, "/gps", self.qos())

        # subscriber for file paths -----------------------------------------
        self.create_subscription(String, "/video_filepath", self.path_cb, 10)

        self.bridge = CvBridge()
        self.playing = False   # only run one video at a time

    # latched QoS profile (for GPS)
    def qos(self):
        return QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

    # callback: start video thread
    def path_cb(self, msg: String):
        if self.playing:
            return
        threading.Thread(target=self.play_video, args=(msg.data,), daemon=True).start()

    # read video frames and publish
    def play_video(self, filepath):
        self.playing = True
        cap = cv2.VideoCapture(filepath)
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        period = 1.0 / fps

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            stamp = self.get_clock().now().to_msg()

            # image ----------------------------------------------------------
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = "camera"
            self.image_pub.publish(img_msg)

            # gps ------------------------------------------------------------
            fix = NavSatFix()
            fix.header.stamp = stamp
            fix.header.frame_id = "map"
            fix.latitude = 53.01020
            fix.longitude = -0.53434
            fix.altitude = 45.24
            self.gps_pub.publish(fix)

            time.sleep(period)

        cap.release()
        self.playing = False


def main():
    rclpy.init()
    node = VideoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
