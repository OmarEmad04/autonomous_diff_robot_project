#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from collections import deque

class PathNode(Node):

    def __init__(self):
        super().__init__('path_node')

        self.sub = self.create_subscription(
            Int32, '/lane/center_line', self.callback, 10)

        self.pub = self.create_publisher(Int32, '/lane/path', 10)
        self.offset = 0
        self.offset_sub = self.create_subscription(
            Int32, '/lane/center_offset', self.offset_callback, 10)

        self.buffer = deque(maxlen=5)  # Adjust if needed
        self.get_logger().info("Path Node Started")

    def callback(self, msg):
        if msg.data == -1:  # No line detected
            self.pub.publish(msg)
            self.buffer.clear()  # Clear buffer when no center detected
            return

        self.buffer.append(msg.data)

        # Use weighted average of recent positions (more weight to the most recent ones)
        if len(self.buffer) > 1:
            smooth_center = sum(self.buffer) / len(self.buffer)  # Weighted average can be added here if needed
        else:
            smooth_center = self.buffer[-1]

        out = Int32()
        #out.data = int(smooth_center)
        out.data = int(smooth_center + self.offset)
        self.pub.publish(out)
    
    def offset_callback(self, msg):
        self.offset = msg.data



def main():
    rclpy.init()
    node = PathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
