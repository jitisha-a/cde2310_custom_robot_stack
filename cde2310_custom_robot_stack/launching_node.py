#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool


class LaunchingNode(Node):
    def __init__(self):
        super().__init__('launching_node')

        self.current_mode = 'IDLE'
        self.has_launched = False

        self.mode_sub = self.create_subscription(
            String,
            '/robot_mode',
            self.mode_callback,
            10
        )

        self.launch_done_pub = self.create_publisher(Bool, '/launch_done', 10)
        self.timer = self.create_timer(0.2, self.loop)

        self.get_logger().info('Launching node started.')

    def mode_callback(self, msg: String):
        self.current_mode = msg.data
        if self.current_mode != 'LAUNCH':
            self.has_launched = False

    def loop(self):
        if self.current_mode != 'LAUNCH':
            return

        if self.has_launched:
            return

        self.get_logger().info('LAUNCH mode active. Trigger launcher here.')
        time.sleep(1.0)

        done = Bool()
        done.data = True
        self.launch_done_pub.publish(done)

        self.has_launched = True
        self.get_logger().info('Launch complete.')

def main(args=None):
    rclpy.init(args=args)
    node = LaunchingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
