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
    particle_filter_share = get_package_share_directory('particle_filter')

    default_localize_config = os.path.join(
        particle_filter_share,
        'config',
        'localize.yaml',
    )
    default_map_yaml = '/home/nvidia/spring26_ws/maps/apt_floor_2.yaml'
    vesc_imu_fusion_config = os.path.join(
        f1tenth_share,
        'config',
        'vesc_imu_fusion.yaml',
    )
    default_mux_config = os.path.join(
        f1tenth_share,
        'config',
        'mux.yaml',
    )

    localize_config_la = DeclareLaunchArgument(
        'localize_config',
        default_value=default_localize_config,
        description='Path to particle_filter localization config yaml',
    )
    map_yaml_la = DeclareLaunchArgument(
        'map_yaml',
        default_value=default_map_yaml,
        description='Full path to occupancy map yaml file used by particle_filter map_server',
    )
    vesc_driver_log_level_la = DeclareLaunchArgument(
        'vesc_driver_log_level',
        default_value='warn',
        description='Log level for vesc_driver_node (debug, info, warn, error, fatal)',
    )
    mux_config_la = DeclareLaunchArgument(
        'mux_config',
        default_value=default_mux_config,
        description='Ackermann mux config file path',
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
            'mux_config': LaunchConfiguration('mux_config'),
            'launch_usb_imu': 'true',
            # Keep base_link->laser static TF for a connected map/base/laser tree.
            'launch_static_tf': 'true',
            'vesc_driver_log_level': LaunchConfiguration('vesc_driver_log_level'),
        }.items(),
    )

    imu_odom_fusion_node = Node(
        package='f1tenth_stack',
        executable='imu_odom_fusion_node',
        name='imu_odom_fusion_node',
        output='screen',
        parameters=[
            {
                'imu_topic': '/sensors/imu/raw',
                'wheel_odom_topic': '/odom',
                'fused_odom_topic': '/odometry/imu_fused',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                # particle_filter publishes map->base_link in this mode.
                'publish_tf': False,
                'linear_speed_scale': LaunchConfiguration('imu_linear_speed_scale'),
                'use_first_imu_as_zero': True,
                'yaw_offset_rad': LaunchConfiguration('imu_yaw_offset_rad'),
                'yaw_alpha': LaunchConfiguration('imu_yaw_alpha'),
                'max_dt_sec': 0.2,
            }
        ],
    )

    particle_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                particle_filter_share,
                'launch',
                'localize_launch.py',
            )
        ),
        launch_arguments={
            'localize_config': LaunchConfiguration('localize_config'),
            'map_yaml': LaunchConfiguration('map_yaml'),
            'odometry_topic': '/odometry/imu_fused',
        }.items(),
    )

    print_usage_instructions = LogInfo(
        msg='Particle filter GPU localization mode started.\n'
            'Edit particle_filter localize config to select map and GPU options.\n'
            'Odometry source: /odometry/imu_fused (IMU + wheel-odom fusion).\n'
            'Default config: ' + default_localize_config + '\n'
            'In RViz, use "2D Pose Estimate" to initialize PF.\n'
    )

    return LaunchDescription(
        [
            localize_config_la,
            map_yaml_la,
            vesc_driver_log_level_la,
            mux_config_la,
            imu_yaw_offset_la,
            imu_yaw_alpha_la,
            imu_linear_speed_scale_la,
            bringup_launch,
            imu_odom_fusion_node,
            particle_filter_launch,
            print_usage_instructions,
        ]
    )
