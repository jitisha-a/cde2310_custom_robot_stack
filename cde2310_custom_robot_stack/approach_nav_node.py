#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class ApproachNavNode(Node):
    def __init__(self):
        super().__init__('approach_nav_node')

        self.current_mode = 'EXPLORE'
        self.latest_goal = None
        self.goal_sent = False
        self.goal_in_progress = False

        self.mode_sub = self.create_subscription(
            String, '/robot_mode', self.mode_callback, 10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped, '/coarse_dock_goal', self.goal_callback, 10
        )

        self.approach_done_pub = self.create_publisher(Bool, '/approach_done', 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.timer = self.create_timer(0.2, self.loop)

        self.get_logger().info('Approach nav node started.')

    def mode_callback(self, msg):
        self.current_mode = msg.data
        if self.current_mode != 'APPROACH':
            self.goal_sent = False
            self.goal_in_progress = False

    def goal_callback(self, msg):
        self.latest_goal = msg

    def loop(self):
        if self.current_mode != 'APPROACH':
            return

        if self.latest_goal is None:
            return

        if self.goal_sent or self.goal_in_progress:
            return

        self.send_nav_goal(self.latest_goal)

    def send_nav_goal(self, pose_msg: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not available yet.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg

        self.get_logger().info(
            f'Sending coarse approach goal: x={pose_msg.pose.position.x:.2f}, '
            f'y={pose_msg.pose.position.y:.2f}'
        )

        self.goal_sent = True
        self.goal_in_progress = True

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Approach goal rejected.')
            self.goal_in_progress = False
            self.goal_sent = False
            return

        self.get_logger().info('Approach goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        self.goal_in_progress = False

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Approach goal reached.')
            self.approach_done_pub.publish(Bool(data=True))
        else:
            self.get_logger().warn(f'Approach goal failed with status {result.status}')
            self.goal_sent = False


def main(args=None):
    rclpy.init(args=args)
    node = ApproachNavNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
