#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cde2310_custom_robot_stack',
            executable='supervisor_node',
            output='screen'
        ),
        Node(
            package='cde2310_custom_robot_stack',
            executable='marker_mapper_node',
            output='screen'
        ),
        Node(
            package='cde2310_custom_robot_stack',
            executable='approach_nav_node',
            output='screen'
        ),
        Node(
            package='cde2310_custom_robot_stack',
            executable='frontier_node',
            output='screen'
        ),
        Node(
            package='cde2310_custom_robot_stack',
            executable='docking_node',
            output='screen'
        ),
    ])
