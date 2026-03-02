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
    vesc_imu_fusion_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc_imu_fusion.yaml',
    )

    imu_topic_la = DeclareLaunchArgument(
        'imu_topic',
        default_value='/sensors/imu',
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
                get_package_share_directory('f1tenth_stack'),
                'launch',
                'bringup_launch.py',
            )
        ),
        launch_arguments={
            'launch_vesc_to_odom': 'true',
            'vesc_config': vesc_imu_fusion_config,
            'motor_speed_output_topic': 'commands/motor/unclipped_speed',
        }.items(),
    )

    speed_clipper_node = Node(
        package='f1tenth_stack',
        executable='speed_clipper',
        name='mapping_speed_clipper',
        parameters=[
            {
                'input_topic': 'commands/motor/unclipped_speed',
                'output_topic': 'commands/motor/speed',
                'min_value': -2500.0,
                'max_value': 2500.0,
            }
        ],
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

    scanmatching_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('f1tenth_stack'),
                'launch',
                'scanmatching_slam_launch.py',
            )
        )
    )

    print_usage_instructions = LogInfo(
        msg='Scan-matching mapping mode with IMU-fused odom orientation enabled.\n'
            'To save the resultant map, keep this session running, open a new '
            'terminal and run:\n'
            '\tros2 run nav2_map_server map_saver_cli -f your_map_file_name '
            '--ros-args -p map_subscribe_transient_local:=true\n'
    )

    return LaunchDescription(
        [
            imu_topic_la,
            imu_yaw_offset_la,
            imu_yaw_alpha_la,
            imu_linear_speed_scale_la,
            bringup_launch,
            speed_clipper_node,
            imu_odom_fusion_node,
            scanmatching_slam_launch,
            print_usage_instructions,
        ]
    )
