#!/usr/bin/env python3

#NEGLECTED FOR NOW
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

import cv2
import numpy as np
from cv_bridge import CvBridge


class ObstacleAvoid(Node):

    def __init__(self):
        super().__init__('obstacle_avoid_node')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/camera/image/image_raw',
            self.image_callback,
            10
        )

        self.offset_pub = self.create_publisher(
            Int32, '/lane/avoid_offset', 10)

        self.cooldown = 0   # frame-based cooldown
        self.active_offset = 0
        self.hold_frames = 0
        self.get_logger().info("Obstacle Avoidance Node Started")

    def image_callback(self, msg):

        # Hold current action
        if self.hold_frames > 0:
            self.hold_frames -= 1
            out = Int32()
            out.data = self.active_offset
            self.offset_pub.publish(out)
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = frame.shape
        mid = w // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        roi_y = int(h * 0.82)
        hsv_roi = hsv[roi_y:h, :]

        # RED
        red1 = cv2.inRange(hsv_roi, (0, 90, 80), (10, 255, 255))
        red2 = cv2.inRange(hsv_roi, (170, 90, 80), (180, 255, 255))
        red_mask = red1 | red2

        contours, _ = cv2.findContours(
            red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            if cv2.contourArea(c) < 250:
                continue
            print("Red obstacle detected")
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            obstacle_error = cx - mid
            
        # GREEN
        green_mask = cv2.inRange(
            hsv_roi, (45, 80, 80), (85, 255, 255))

        contours, _ = cv2.findContours(
            green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            if cv2.contourArea(c) < 250:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])

            if cx > mid:
                self.active_offset = +80
                self.hold_frames = 40
                self.get_logger().info("LOCKED: GREEN obstacle → RIGHT")
                return


def main():
    rclpy.init()
    node = ObstacleAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()