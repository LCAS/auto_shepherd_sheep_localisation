#!/usr/bin/env python3

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from cv_bridge import CvBridge
import cv2
import math

from sensor_msgs.msg import Image, NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Vector3Stamped

from auto_shepherd_sheep_localisation_ros2.detection_process.modules.sheepdetectROS import SheepDetectROS
from auto_shepherd_sheep_localisation_ros2.detection_process.modules.config import *
from ultralytics import YOLO


class SheepDetector(Node):
    def __init__(self):
        super().__init__("sheep_detector")
        print('\n\n\n')
        self.get_logger().info("🐑 Sheep Detector Node Starting...")

        # latest GPS fix (updated by gps_cb)
        self.gps = None
        
        # latest drone attitude (yaw/pitch/roll)
        self.attitude = None
        
        # latest gimbal orientation
        self.gimbal = None
        
        # latest camera data
        self.camera = None

        # publisher for the outgoing Path and Labelled image
        self.path_pub = self.create_publisher(Path, "/sheep/poses", 10)
        self.boxes_pub = self.create_publisher(Image, "/sheep/labelled_img", 10)

        # subscribe to latched GPS and live image topics
        self.create_subscription(NavSatFix, "/drone/gps", self.gps_cb, self.qos())
        self.create_subscription(Image, "/drone/image", self.image_cb, 10)
        
        # subscribe to drone attitude and gimbal for accurate GPS conversion
        self.create_subscription(Vector3Stamped, "/drone/attitude", self.attitude_cb, 10)
        self.create_subscription(Vector3Stamped, "/drone/gimbal", self.gimbal_cb, 10)

        # Identify weights filepath - use env vars if set, otherwise use config defaults
        yolo_weights = os.getenv('YOLO_WEIGHTS')
        yolo_weights_path = yolo_weights if yolo_weights else YOLO_WEIGHTS_SHEEP
        
        yolo_tracker = os.getenv('YOLO_TRACKER')
        yolo_tracker_path = yolo_tracker if yolo_tracker else YOLO_TRACKER

        self.SD = SheepDetectROS(yolo_weights_path, yolo_tracker_path,
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
            durability=QoSDurabilityPolicy.VOLATILE,
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




        # Cluster detections by proximity and publish cluster centroids
        if len(sheep_ids) > 0:
            clusters = self.cluster_sheep([poses[i]['position'] for i in range(len(sheep_ids))])
            cluster_path = Path()
            cluster_path.header.stamp = msg.header.stamp
            cluster_path.header.frame_id = "map"
            for idx, (lat_c, lon_c, count) in enumerate(clusters, start=1):
                ps = PoseStamped()
                ps.header.stamp = msg.header.stamp
                ps.header.frame_id = f"cluster_{idx}"
                ps.pose.position.x = lat_c
                ps.pose.position.y = lon_c
                ps.pose.position.z = float(count)  # store cluster size in z
                cluster_path.poses.append(ps)
            self.cluster_pub.publish(cluster_path)

    def cluster_sheep(self, positions, radius_m=15.0):
        """Simple agglomerative clustering by distance (meters). positions: list of dicts with x(lat), y(lon)."""
        clusters = []
        for pos in positions:
            lat = pos['x']
            lon = pos['y']
            placed = False
            for c in clusters:
                c_lat, c_lon, c_n = c
                # rough meters per degree
                lat_m = 111320.0
                lon_m = 40008000.0 * math.cos(math.radians(c_lat)) / 360.0
                d_lat_m = (lat - c_lat) * lat_m
                d_lon_m = (lon - c_lon) * lon_m
                dist = math.sqrt(d_lat_m * d_lat_m + d_lon_m * d_lon_m)
                if dist <= radius_m:
                    # update centroid
                    new_n = c_n + 1
                    c[0] = (c_lat * c_n + lat) / new_n
                    c[1] = (c_lon * c_n + lon) / new_n
                    c[2] = new_n
                    placed = True
                    break
            if not placed:
                clusters.append([lat, lon, 1])
        return clusters


def main():
    rclpy.init()
    node = SheepDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
