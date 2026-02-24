# MIT License
#
# Copyright (c) 2026 Bala Kolanu
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc.yaml',
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('f1tenth_stack'),
                'launch',
                'bringup_launch.py',
            )
        ),
        launch_arguments={
            'launch_vesc_to_odom': 'true',
        }.items(),
    )

    scanmatching_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('f1tenth_stack'),
                'launch',
                'scanmatching_slam_launch.py',
            )
        )
    )

    scanmatching_reference_pose_node = Node(
        package='f1tenth_stack',
        executable='tf_pose_reference_publisher',
        name='scanmatching_reference_pose_publisher',
        output='screen',
        parameters=[
            {
                'source_frame': 'map',
                'target_frame': 'base_link',
                'output_topic': '/scanmatching_odom/pose',
                'publish_frequency': 20.0,
            },
        ],
    )

    linear_odom_calibrator_node = Node(
        package='f1tenth_stack',
        executable='linear_odom_calibrator',
        name='linear_odom_calibrator',
        output='screen',
        parameters=[
            {
                'odom_topic': '/odom',
                'reference_topic': '/scanmatching_odom/pose',
                'sampling_frequency': 20.0,
                'max_yaw_rate': 0.10,
                'max_pair_jump': 0.25,
                'min_pair_step': 0.002,
                'min_reference_distance': 2.0,
                'vesc_config_path': vesc_config,
            },
        ],
    )

    print_usage_instructions = LogInfo(
        msg='Odom calibration mode started.\n'
            'Use /odom as wheel-odom estimate and /scanmatching_odom/pose as reference.\n'
            'Reference is derived from TF map->base_link.\n'
            'Drive mostly straight while steering is centered.\n'
            'Stop this launch with Ctrl+C to print calibrated linear odom values.\n'
    )

    return LaunchDescription(
        [
            bringup_launch,
            scanmatching_slam_launch,
            scanmatching_reference_pose_node,
            linear_odom_calibrator_node,
            print_usage_instructions,
        ]
    )
