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
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    f1tenth_share = get_package_share_directory('f1tenth_stack')
    vesc_imu_fusion_config = os.path.join(
        f1tenth_share,
        'config',
        'vesc_imu_fusion.yaml',
    )
    imu_orientation_only_config = os.path.join(
        f1tenth_share,
        'config',
        'imu_odom_orientation_only.yaml',
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
        description='Scale factor applied to wheel odom linear speed in orientation + odom fusion',
    )
    scanmatched_imu_odom_topic_la = DeclareLaunchArgument(
        'scanmatched_imu_odom_topic',
        default_value='/odometry/scanmatched_imu',
        description='Output topic for scan-matched translation with IMU orientation',
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
            'imu_fusion_config': imu_orientation_only_config,
            'motor_speed_output_topic': 'commands/motor/unclipped_speed',
            'launch_usb_imu': 'false',
            'launch_bno085_i2c': 'true',
            'launch_imu_fusion': 'true',
            'imu_topic': LaunchConfiguration('imu_topic'),
            'imu_fused_odom_topic': '/odometry/imu_fused',
            'imu_fusion_publish_tf': 'true',
            'imu_yaw_offset_rad': LaunchConfiguration('imu_yaw_offset_rad'),
            'imu_yaw_alpha': LaunchConfiguration('imu_yaw_alpha'),
            'imu_linear_speed_scale': LaunchConfiguration('imu_linear_speed_scale'),
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
                'output_topic': '/scanmatching_tf/pose',
                'publish_frequency': 20.0,
            },
        ],
    )

    scanmatching_imu_odom_node = Node(
        package='f1tenth_stack',
        executable='scanmatching_imu_odom_node',
        name='scanmatching_imu_odom_node',
        output='screen',
        parameters=[
            {
                'pose_topic': '/scanmatching_tf/pose',
                'imu_topic': LaunchConfiguration('imu_topic'),
                'output_topic': LaunchConfiguration('scanmatched_imu_odom_topic'),
                'output_frame': 'map',
                'base_frame': 'base_link',
                'imu_frame_id': 'imu_frame',
                'publish_tf': False,
                'yaw_offset_rad': LaunchConfiguration('imu_yaw_offset_rad'),
                'yaw_alpha': LaunchConfiguration('imu_yaw_alpha'),
                'use_imu_angular_velocity': True,
            }
        ],
    )

    print_usage_instructions = LogInfo(
        msg=[
            'Scan-matching mapping mode with wheel odom + IMU orientation fusion enabled '
            '(accelerometer fusion disabled).\n'
            'Final mapping odom topic: ',
            LaunchConfiguration('scanmatched_imu_odom_topic'),
            ' (scan-matched translation + IMU orientation).\n'
            'To save the resultant map, keep this session running, open a new '
            'terminal and run:\n'
            '\tros2 run nav2_map_server map_saver_cli -f your_map_file_name '
            '--ros-args -p map_subscribe_transient_local:=true\n',
        ]
    )

    return LaunchDescription(
        [
            imu_topic_la,
            imu_yaw_offset_la,
            imu_yaw_alpha_la,
            imu_linear_speed_scale_la,
            scanmatched_imu_odom_topic_la,
            bringup_launch,
            speed_clipper_node,
            scanmatching_slam_node,
            scanmatching_reference_pose_node,
            scanmatching_imu_odom_node,
            print_usage_instructions,
        ]
    )
