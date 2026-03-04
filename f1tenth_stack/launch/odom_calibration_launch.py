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
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    f1tenth_share = get_package_share_directory('f1tenth_stack')
    vesc_imu_fusion_config = os.path.join(
        f1tenth_share,
        'config',
        'vesc_imu_fusion.yaml',
    )
    scanmatching_slam_config = os.path.join(
        f1tenth_share,
        'config',
        'slam_toolbox_scanmatching.yaml',
    )

    imu_topic_la = DeclareLaunchArgument(
        'imu_topic',
        default_value='/sensors/imu/raw',
        description='IMU topic used for yaw fusion',
    )
    imu_yaw_offset_la = DeclareLaunchArgument(
        'imu_yaw_offset_rad',
        default_value='0.0',
        description='Static yaw offset added to IMU heading (radians)',
    )
    imu_yaw_alpha_la = DeclareLaunchArgument(
        'imu_yaw_alpha',
        default_value='0.3',
        description='IMU yaw smoothing factor in [0,1], higher tracks faster',
    )
    imu_linear_speed_scale_la = DeclareLaunchArgument(
        'imu_linear_speed_scale',
        default_value='1.0',
        description='Scale factor applied to wheel odom linear speed in fusion',
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                f1tenth_share,
                'launch',
                'bringup_launch.py',
            )
        ),
        launch_arguments={
            'launch_vesc_to_odom': 'true',
            'vesc_config': vesc_imu_fusion_config,
            'launch_usb_imu': 'true',
        }.items(),
    )

    imu_odom_fusion_node = Node(
        package='f1tenth_stack',
        executable='imu_odom_fusion_node',
        name='imu_odom_fusion_node',
        output='screen',
        parameters=[
            {
                'imu_topic': LaunchConfiguration('imu_topic'),
                'wheel_odom_topic': '/odom',
                'fused_odom_topic': '/odometry/imu_fused',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
                'linear_speed_scale': LaunchConfiguration('imu_linear_speed_scale'),
                'use_first_imu_as_zero': True,
                'yaw_offset_rad': LaunchConfiguration('imu_yaw_offset_rad'),
                'yaw_alpha': LaunchConfiguration('imu_yaw_alpha'),
                'max_dt_sec': 0.2,
            }
        ],
    )

    scanmatching_slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_node',
        output='screen',
        parameters=[
            scanmatching_slam_config,
            {
                'map_frame': 'map',
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'transform_publish_period': 0.02,
                'use_sim_time': False,
            },
        ],
        remappings=[
            ('pose', '/scanmatching_odom/pose'),
        ],
        arguments=['--ros-args', '--log-level', 'warn'],
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
                'odom_topic': '/odometry/imu_fused',
                'reference_topic': '/scanmatching_odom/pose',
                'sampling_frequency': 20.0,
                'max_yaw_rate': 0.10,
                'max_pair_jump': 0.25,
                'min_pair_step': 0.002,
                'min_reference_distance': 2.0,
                'vesc_config_path': vesc_imu_fusion_config,
            },
        ],
    )

    print_usage_instructions = LogInfo(
        msg='Odom calibration mode started.\n'
            'Use /odometry/imu_fused as odom estimate and /scanmatching_odom/pose as reference.\n'
            'Reference is derived from TF map->base_link.\n'
            'Drive mostly straight while steering is centered.\n'
            'Stop this launch with Ctrl+C to print calibrated linear odom values.\n'
    )

    return LaunchDescription(
        [
            imu_topic_la,
            imu_yaw_offset_la,
            imu_yaw_alpha_la,
            imu_linear_speed_scale_la,
            bringup_launch,
            imu_odom_fusion_node,
            scanmatching_slam_node,
            scanmatching_reference_pose_node,
            linear_odom_calibrator_node,
            print_usage_instructions,
        ]
    )
