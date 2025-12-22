#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Int32

import cv2
import numpy as np
from cv_bridge import CvBridge


class LaneVision(Node):

    def __init__(self):
        super().__init__('lane_vision_node')
        

        # QoS for camera topic
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribe to camera
        self.sub = self.create_subscription(
            Image,
            '/camera/image/image_raw',
            self.image_callback,
            qos
        )
    
        # Publishers
        self.center_pub = self.create_publisher(Int32, '/lane/center_line', 10)
        self.debug_pub = self.create_publisher(Image, '/lane/debug_image', 10)

        self.bridge = CvBridge()
        self.get_logger().info("Lane Vision Node Started")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = frame.shape

        # ==============================
        # 1. Convert to HSV
        # ==============================
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # White color range (Gazebo-friendly)
        lower_white = np.array([0, 0, 120])
        upper_white = np.array([180, 80, 255])

        mask = cv2.inRange(hsv, lower_white, upper_white)
        self.debug_pub.publish(
            self.bridge.cv2_to_imgmsg(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), 'bgr8')
        )


        # ==============================
        # 2. ROI (bottom 40%)
        # ==============================
        roi_y = int(h * 0.6)
        roi = mask[roi_y:h, :]

        # ==============================
        # 3. Morphology (clean noise)
        # ==============================
        kernel = np.ones((5, 5), np.uint8)
        roi = cv2.morphologyEx(roi, cv2.MORPH_OPEN, kernel)
        roi = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)
        
        #addition: Apply Gaussian blur to the ROI to reduce noise
        roi = cv2.GaussianBlur(roi, (5, 5), 0)


        # ==============================
        # 4. Find contours
        # ==============================
        
        contours, _ = cv2.findContours(
            roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter small contours
        contours = [c for c in contours if cv2.contourArea(c) > 300] #it was 150

        self.get_logger().info(
            f"Contours after HSV filter: {len(contours)}",
            throttle_duration_sec=1.0
        )

        # ==============================
        # 5. Debug image
        # ==============================
        debug = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Draw ROI box
        cv2.rectangle(
            debug,
            (0, roi_y),
            (w, h),
            (255, 0, 0),
            2
        )

        # ==============================
        # 6. Compute lane center (robust)
        # ==============================
        center_msg = Int32()
        center_msg.data = -1

        mid = w // 2

        left_best = None   # (area, cx)
        right_best = None  # (area, cx)

        for c in contours:
            area = cv2.contourArea(c)
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])

            if cx < mid:
                if (left_best is None) or (area > left_best[0]):
                    left_best = (area, cx)
            else:
                if (right_best is None) or (area > right_best[0]):
                    right_best = (area, cx)

        if left_best and right_best:
            left_x = left_best[1]
            right_x = right_best[1]
            lane_center = (left_x + right_x) // 2 

            center_msg.data = lane_center

            cv2.line(debug, (mid, h), (mid, roi_y), (0, 0, 255), 2)         # image mid
            cv2.line(debug, (left_x, h), (left_x, roi_y), (255, 0, 0), 3)
            cv2.line(debug, (right_x, h), (right_x, roi_y), (255, 0, 0), 3)
            cv2.line(debug, (lane_center, h), (lane_center, roi_y), (0, 255, 0), 3)


        # ==============================
        # 7. Publish
        # ==============================
        self.center_pub.publish(center_msg)
        self.debug_pub.publish(
            self.bridge.cv2_to_imgmsg(debug, 'bgr8')
        )

def main():
    rclpy.init()
    node = LaneVision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
