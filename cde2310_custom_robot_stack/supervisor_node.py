#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')

        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)
        self.launch_cmd_pub = self.create_publisher(Bool, '/launch_cmd', 10)

        self.coarse_goal_ready_sub = self.create_subscription(
            Bool, '/coarse_goal_ready', self.coarse_goal_ready_callback, 10
        )

        self.approach_done_sub = self.create_subscription(
            Bool, '/approach_done', self.approach_done_callback, 10
        )

        self.docking_done_sub = self.create_subscription(
            Bool, '/docking_done', self.docking_done_callback, 10
        )

        self.launch_done_sub = self.create_subscription(
            Bool, '/launch_done', self.launch_done_callback, 10
        )

        self.current_mode = 'EXPLORE'
        self.last_published_mode = None
        self.launch_command_sent = False

        self.timer = self.create_timer(0.2, self.loop)

        self.get_logger().info('Supervisor started. Initial mode = EXPLORE')

    def loop(self):
        if self.current_mode != self.last_published_mode:
            self.get_logger().info(f'Switching mode -> {self.current_mode}')
            self.last_published_mode = self.current_mode

        self.mode_pub.publish(String(data=self.current_mode))

        if self.current_mode == 'LAUNCH' and not self.launch_command_sent:
            self.get_logger().info('Publishing /launch_cmd = True')
            self.launch_cmd_pub.publish(Bool(data=True))
            self.launch_command_sent = True

    def coarse_goal_ready_callback(self, msg: Bool):
        if msg.data and self.current_mode == 'EXPLORE':
            self.get_logger().info('Coarse docking goal ready. Switching to APPROACH.')
            self.current_mode = 'APPROACH'

    def approach_done_callback(self, msg: Bool):
        if msg.data and self.current_mode == 'APPROACH':
            self.get_logger().info('Approach complete. Switching to DOCK.')
            self.current_mode = 'DOCK'

    def docking_done_callback(self, msg: Bool):
        if msg.data and self.current_mode == 'DOCK':
            self.get_logger().info('Docking complete. Switching to LAUNCH.')
            self.current_mode = 'LAUNCH'
            self.launch_command_sent = False

    def launch_done_callback(self, msg: Bool):
        if msg.data and self.current_mode == 'LAUNCH':
            self.get_logger().info('Launch complete. Switching to IDLE.')
            self.current_mode = 'IDLE'
            self.launch_command_sent = False


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
