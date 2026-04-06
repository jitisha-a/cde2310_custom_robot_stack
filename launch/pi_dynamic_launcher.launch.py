#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cde2310_custom_robot_stack',
            executable='dynamic_launcher_hw_node',
            output='screen'
        ),
    ])
