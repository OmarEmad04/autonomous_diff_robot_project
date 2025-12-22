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

        self.offset_sub = self.create_subscription(
            Int32, '/lane/avoid_offset', self.offset_callback, 10)

        self.pub = self.create_publisher(Int32, '/lane/path', 10)

        self.offset = 0
        self.buffer = deque(maxlen=5)

        self.get_logger().info("Path Node Started")


    def offset_callback(self, msg):
        self.offset = msg.data


    def callback(self, msg):
        if msg.data == -1:
            self.pub.publish(msg)
            self.buffer.clear()
            return

        self.buffer.append(msg.data)
        smooth = sum(self.buffer) / len(self.buffer)

        out = Int32()
        out.data = int(smooth + self.offset)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()