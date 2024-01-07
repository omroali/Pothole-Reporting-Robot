#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        # setting up the node name
        super().__init__("first_node")
        self.get_logger().info("Hello from ROS2")  # logging in ros2


def main(args=None):
    # initiallising ros communications
    rclpy.init(args=args)

    # creating the node
    node = MyNode()

    # keeping the node alive until it's killed
    rclpy.spin(node)

    # shutting down communications
    rclpy.shutdown()


if __name__ == "__main__":
    main()
