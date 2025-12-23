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

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub = self.create_subscription(
            Image, '/camera/image/image_raw', self.image_callback, qos)

        self.center_pub = self.create_publisher(Int32, '/lane/center_line', 10)
        # self.avoid_pub  = self.create_publisher(Int32, '/lane/avoid_offset', 10)
        self.debug_pub  = self.create_publisher(Image, '/lane/debug_image', 10)
        self.finish_pub  = self.create_publisher(Int32, '/finish_line', 10)

        self.bridge = CvBridge()
        self.get_logger().info("Lane Vision Node Started")


    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = frame.shape
        mid = w // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # =====================
        # WHITE LANE MASK
        # =====================
        white_mask = cv2.inRange(
            hsv,
            np.array([0, 0, 120]),
            np.array([180, 80, 255])
        )

        roi_y = int(h * 0.6)
        roi = white_mask[roi_y:h, :]

        kernel = np.ones((5, 5), np.uint8)
        roi = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) > 300]

        center_msg = Int32()
        center_msg.data = -1

        left = right = None

        for c in contours:
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            if cx < mid:
                left = cx
            else:
                right = cx

        if left and right:
            center_msg.data = (left + right) // 2
        obstacle_target = center_msg.data

        

        # =====================
        # OBSTACLE DETECTION
        # =====================
        avoid = Int32()
        avoid.data = 0
        
        # RED
        red_mask1 = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255))
        red_mask2 = cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
        red_mask = red_mask1 | red_mask2

        # GREEN
        green_mask = cv2.inRange(hsv, (40, 70, 70), (80, 255, 255))

        for mask, color in [(red_mask, "red"), (green_mask, "green")]:
            roi_obs = mask[roi_y:h, :]
            cnts, _ = cv2.findContours(
                roi_obs, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for c in cnts:
                if cv2.contourArea(c) < 30000:
                    continue
                
                M = cv2.moments(c)
                if M["m00"] == 0:
                    print("m000 is zero")
                    continue
                    

                cx = int(M["m10"] / M["m00"])
                
                if color == "red":# and cx > mid:
                    self.get_logger().info("Red obstacle right → move left")
                    obstacle_target = cx - mid
                    if obstacle_target< 0:
                        obstacle_target = 0
                elif color == "green": #and cx < mid:
                    self.get_logger().info("Green obstacle Left → move right")
                    obstacle_target = cx + mid
                    if obstacle_target> 639:
                        obstacle_target = 639
                    obstacle_target = 600
                    # move RIGHT
                else:
                    print(f"{color} obstacle ignored")
                #obstacle_target = obstacle_target * (cv2.contourArea(c)/300) # Weight by size
        # self.avoid_pub.publish(avoid)
        blending_ratio = 0.2  # Tunable parameter between 0 and 1
        center_msg.data = int(blending_ratio * center_msg.data + (1 - blending_ratio) * obstacle_target) #ratio is tunable
        print("Final center:", center_msg.data)
        self.center_pub.publish(center_msg)
        self.debug_pub.publish(
            self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        )


def main():
    rclpy.init()
    node = LaneVision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()