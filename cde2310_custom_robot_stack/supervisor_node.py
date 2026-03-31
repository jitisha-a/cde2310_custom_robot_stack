#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool


class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')

        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)

        self.marker_found_sub = self.create_subscription(
            Bool,
            '/target_marker_found',
            self.marker_found_callback,
            10
        )

        self.docking_done_sub = self.create_subscription(
            Bool,
            '/docking_done',
            self.docking_done_callback,
            10
        )

        self.launch_done_sub = self.create_subscription(
            Bool,
            '/launch_done',
            self.launch_done_callback,
            10
        )

        self.current_mode = 'EXPLORE'
        self.last_published_mode = None

        self.timer = self.create_timer(0.2, self.publish_mode)
        self.get_logger().info('Supervisor started. Initial mode = EXPLORE')

    def publish_mode(self):
        if self.current_mode != self.last_published_mode:
            self.get_logger().info(f'Switching mode -> {self.current_mode}')
            self.last_published_mode = self.current_mode

        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    def marker_found_callback(self, msg: Bool):
        if msg.data and self.current_mode == 'EXPLORE':
            self.get_logger().info('Marker found. Switching from EXPLORE to DOCK.')
            self.current_mode = 'DOCK'

    def docking_done_callback(self, msg: Bool):
        if msg.data and self.current_mode == 'DOCK':
            self.get_logger().info('Docking done. Switching from DOCK to LAUNCH.')
            self.current_mode = 'LAUNCH'

    def launch_done_callback(self, msg: Bool):
        if msg.data and self.current_mode == 'LAUNCH':
            self.get_logger().info('Launch done. Switching to IDLE.')
            self.current_mode = 'IDLE'


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
