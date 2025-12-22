#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class LaneController(Node):

    def __init__(self):
        super().__init__('lane_controller')

        self.sub = self.create_subscription(
            Int32, '/lane/path', self.callback, 10)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.image_center = 320  # adjust if needed
        self.kp = 0.002
        self.max_speed = 0.2
        self.missed_frames = 0  # Counter to track consecutive lost center detections

        self.get_logger().info("Controller Node Started")

    def callback(self, msg):
        cmd = Twist()

        if msg.data == -1:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            return


        error = self.image_center - msg.data
        # Deadband (ignore small jitter)
        if abs(error) < 8:  # Tolerance for small error
            error = 0

        # Adjust speed based on error magnitude
        distance_from_center = abs(error)
        speed = max(self.max_speed - (distance_from_center * 0.001), 0.03)  # Adjust speed with error

        cmd.linear.x = speed
        cmd.angular.z = -self.kp * error  # Proportional control

        # Clamp angular speed to avoid overshooting
        cmd.angular.z = max(min(cmd.angular.z, 0.6), -0.6)

        self.pub.publish(cmd)
        
    def offset_callback(self, msg):
        self.avoid_offset = msg.data



def main():
    rclpy.init()
    node = LaneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
