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
    default_sensors_config = os.path.join(
        f1tenth_share,
        'config',
        'sensors.yaml',
    )
    imu_fusion_pf_config = os.path.join(
        f1tenth_share,
        'config',
        'imu_odom_fusion_pf.yaml',
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
    sensors_config_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=default_sensors_config,
        description='LiDAR/other sensor config file path passed to bringup',
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
    imu_wheel_speed_alpha_la = DeclareLaunchArgument(
        'imu_wheel_speed_alpha',
        default_value='0.85',
        description='Weight assigned to wheel speed versus IMU-accelerated prediction in fusion',
    )
    secondary_imu_topic_la = DeclareLaunchArgument(
        'secondary_imu_topic',
        default_value='',
        description='Optional secondary IMU topic fused with the primary IMU heading',
    )
    secondary_imu_frame_id_la = DeclareLaunchArgument(
        'secondary_imu_frame_id',
        default_value='base_link',
        description='Fallback frame id used for the secondary IMU when messages omit header.frame_id',
    )
    secondary_imu_weight_la = DeclareLaunchArgument(
        'secondary_imu_weight',
        default_value='0.35',
        description='Relative heading weight assigned to the secondary IMU when both are healthy',
    )
    secondary_imu_angular_velocity_weight_la = DeclareLaunchArgument(
        'secondary_imu_angular_velocity_weight',
        default_value='0.5',
        description='Relative yaw-rate weight assigned to the secondary IMU when both are healthy',
    )
    secondary_imu_timeout_sec_la = DeclareLaunchArgument(
        'secondary_imu_timeout_sec',
        default_value='0.25',
        description='Maximum age difference allowed before a secondary IMU sample is treated as stale',
    )
    prefer_secondary_imu_on_yaw_disagreement_la = DeclareLaunchArgument(
        'prefer_secondary_imu_on_yaw_disagreement',
        default_value='false',
        description='Prefer the secondary IMU when dual IMUs strongly disagree on yaw',
    )
    imu_fusion_config_la = DeclareLaunchArgument(
        'imu_fusion_config',
        default_value=imu_fusion_pf_config,
        description='Path to IMU/wheel odom fusion config yaml used for PF bringup',
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
            'launch_hall_odom': 'false',
            'vesc_config': vesc_imu_fusion_config,
            'mux_config': LaunchConfiguration('mux_config'),
            'sensors_config': LaunchConfiguration('sensors_config'),
            'launch_usb_imu': 'false',
            'launch_bno085_i2c': 'true',
            'launch_imu_fusion': 'true',
            'imu_fusion_config': LaunchConfiguration('imu_fusion_config'),
            'imu_topic': '/sensors/imu/raw',
            'imu_fused_odom_topic': '/odometry/imu_fused',
            'imu_fusion_publish_tf': 'false',
            'imu_yaw_offset_rad': LaunchConfiguration('imu_yaw_offset_rad'),
            'imu_yaw_alpha': LaunchConfiguration('imu_yaw_alpha'),
            'imu_linear_speed_scale': LaunchConfiguration('imu_linear_speed_scale'),
            'imu_wheel_speed_alpha': LaunchConfiguration('imu_wheel_speed_alpha'),
            'secondary_imu_topic': LaunchConfiguration('secondary_imu_topic'),
            'secondary_imu_frame_id': LaunchConfiguration('secondary_imu_frame_id'),
            'secondary_imu_weight': LaunchConfiguration('secondary_imu_weight'),
            'secondary_imu_angular_velocity_weight': LaunchConfiguration(
                'secondary_imu_angular_velocity_weight'
            ),
            'secondary_imu_timeout_sec': LaunchConfiguration('secondary_imu_timeout_sec'),
            'prefer_secondary_imu_on_yaw_disagreement': LaunchConfiguration(
                'prefer_secondary_imu_on_yaw_disagreement'
            ),
            # Keep base_link->laser static TF for a connected map/base/laser tree.
            'launch_static_tf': 'true',
            'vesc_driver_log_level': LaunchConfiguration('vesc_driver_log_level'),
        }.items(),
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
            sensors_config_la,
            imu_yaw_offset_la,
            imu_yaw_alpha_la,
            imu_linear_speed_scale_la,
            imu_wheel_speed_alpha_la,
            secondary_imu_topic_la,
            secondary_imu_frame_id_la,
            secondary_imu_weight_la,
            secondary_imu_angular_velocity_weight_la,
            secondary_imu_timeout_sec_la,
            prefer_secondary_imu_on_yaw_disagreement_la,
            imu_fusion_config_la,
            bringup_launch,
            particle_filter_launch,
            print_usage_instructions,
        ]
    )
