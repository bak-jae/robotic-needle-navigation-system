# Copyright 2026 Pusan National University, Advanced Robotics Lab
# SPDX-License-Identifier: LicenseRef-PNU-ARL-Academic-Research-Only-1.0

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("ros2_epos_cmd")) / "config" / "epos.yaml"
    )
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="EPOS bridge ROS parameter YAML file",
            ),
            Node(
                package="ros2_epos_cmd",
                executable="epos_motion_bridge_node",
                name="epos_motion_bridge_node",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
