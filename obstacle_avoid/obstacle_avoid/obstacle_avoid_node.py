#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

import cv2
import numpy as np
from cv_bridge import CvBridge
import time


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

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.offset_pub = self.create_publisher(Int32, '/lane/center_offset', 10)

        self.busy = False   # prevents repeated triggering
        self.get_logger().info("Obstacle Avoidance Node Started")

    def image_callback(self, msg):
        if self.busy:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # -------- RED detection --------
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + \
                   cv2.inRange(hsv, lower_red2, upper_red2)

        # -------- GREEN detection --------
        lower_green = np.array([40, 70, 70])
        upper_green = np.array([80, 255, 255])

        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        if cv2.countNonZero(mask_red) > 800:
            self.get_logger().info("RED obstacle detected → change lane RIGHT")
            self.avoid(direction="right")

        elif cv2.countNonZero(mask_green) > 800:
            self.get_logger().info("GREEN obstacle detected → change lane LEFT")
            self.avoid(direction="left")

    def avoid(self, direction):
        self.busy = True

        cmd = Twist()

        # -------- Rotate --------
        cmd.angular.z = -0.5 if direction == "right" else 0.5
        self.cmd_pub.publish(cmd)
        time.sleep(1.0)

        # -------- Move forward --------
        cmd.angular.z = 0.0
        cmd.linear.x = 0.15
        self.cmd_pub.publish(cmd)
        time.sleep(1.2)

        # -------- Apply lane offset --------
        offset = Int32()
        offset.data = 80 if direction == "right" else -80
        self.offset_pub.publish(offset)

        # -------- Stop override --------
        cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)

        time.sleep(0.5)

        self.busy = False


def main():
    rclpy.init()
    node = ObstacleAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
