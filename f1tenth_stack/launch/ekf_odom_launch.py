"""Launch the opt-in planar EKF odometry node."""

# Copyright (c) 2026 Purdue University
# SPDX-License-Identifier: MIT

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the standalone EKF launch description."""
    default_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'ekf_odom.yaml',
    )
    config_argument = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Planar EKF parameter file',
    )
    ekf_node = Node(
        package='f1tenth_stack',
        executable='ekf_odom_node',
        name='ekf_odom_node',
        output='screen',
        parameters=[LaunchConfiguration('config')],
    )
    return LaunchDescription([config_argument, ekf_node])
