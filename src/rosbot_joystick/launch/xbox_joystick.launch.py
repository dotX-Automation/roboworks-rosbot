#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rosbot_joystick')
    config_file = os.path.join(pkg_share, 'config', 'xbox_joystick.yaml')

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package="rosbot_joystick",
            executable="joystick_node",
            name="rosbot_joystick",
            output="screen",
            parameters=[config_file],
            remappings=[
                ("/joy", "/joy"),
                ("/cmd_vel", "/cmd_vel"),
            ],
        ),
    ])
