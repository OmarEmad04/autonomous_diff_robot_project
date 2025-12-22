#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

import cv2
import numpy as np
from cv_bridge import CvBridge



class LaneFollower(Node):

    def __init__(self):
        super().__init__('lane_follower')

        # Subscribers & Publishers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image/image_raw',
            self.image_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.line_pub = self.create_publisher(
            Marker,
            '/centroid_line',
            10
        )

        self.bridge = CvBridge()

        # Robot parameters (slow & stable)
        self.linear_speed = 0.08
        self.angular_gain = 0.002

        self.get_logger().info('Lane Follower Node Started')

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        self.get_logger().info("Image received")
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        height, width, _ = frame.shape

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Blur to reduce noise
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Binary threshold (white lanes)
        _, binary = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)

        # Region of Interest (bottom part of image)
        roi_height = int(height * 0.6)
        roi = binary[roi_height:height, :]

        # Find contours
        contours, _ = cv2.findContours(
            roi,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # Filter contours by area
        contours = [c for c in contours if cv2.contourArea(c) > 500]

        if len(contours) < 2:
            self.stop_robot()
            return

        # Sort by x position
        contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[0])

        left_contour = contours[0]
        right_contour = contours[-1]

        # Compute centroids
        left_cx = self.get_centroid_x(left_contour)
        right_cx = self.get_centroid_x(right_contour)

        if left_cx is None or right_cx is None:
            self.stop_robot()
            return

        # Adjust for ROI offset
        left_cx_global = left_cx
        right_cx_global = right_cx

        # Lane center
        lane_center = (left_cx_global + right_cx_global) // 2
        image_center = width // 2

        # Control error
        error = image_center - lane_center

        # Command velocities
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_gain * error

        self.cmd_pub.publish(twist)

        # Visualization (optional but useful)
        vis = frame.copy()
        cv2.line(vis, (lane_center, height), (lane_center, roi_height), (0, 255, 0), 3)
        cv2.line(vis, (image_center, height), (image_center, roi_height), (0, 0, 255), 2)

        cv2.imshow("Lane Detection", vis)
        cv2.imshow("Binary", binary)
        cv2.imshow("ROI", roi)

        cv2.waitKey(1)

    def get_centroid_x(self, contour):
        M = cv2.moments(contour)
        self.line_pub.publish(M)
        if M['m00'] == 0:
            return None
        return int(M['m10'] / M['m00'])

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
