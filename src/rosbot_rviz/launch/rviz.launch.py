#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rosbot_rviz")
    rviz_file = os.path.join(pkg_share, "config", "rosbot.rviz")

    return LaunchDescription([
            DeclareLaunchArgument(
                "rviz_config",
                default_value=rviz_file,
                description="Path to the RViz2 configuration file.",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
    ])
