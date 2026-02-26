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

    default_map_yaml = '/home/nvidia/spring26_ws/maps/apt_floor_2.yaml'
    default_amcl_params = os.path.join(
        f1tenth_share,
        'config',
        'amcl_localization.yaml',
    )
    vesc_imu_fusion_config = os.path.join(
        f1tenth_share,
        'config',
        'vesc_imu_fusion.yaml',
    )

    map_yaml_la = DeclareLaunchArgument(
        'map_yaml',
        default_value=default_map_yaml,
        description='Path to occupancy map yaml file',
    )
    amcl_params_la = DeclareLaunchArgument(
        'amcl_params',
        default_value=default_amcl_params,
        description='Path to AMCL parameter yaml file',
    )
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock',
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
                'imu_topic': '/sensors/imu/raw',
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

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'yaml_filename': LaunchConfiguration('map_yaml'),
            }
        ],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            LaunchConfiguration('amcl_params'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }
        ],
    )

    print_usage_instructions = LogInfo(
        msg='AMCL localization mode started.\n'
            'In RViz:\n'
            '  1) Set Fixed Frame to map\n'
            '  2) Map display topic = /map, Durability = Transient Local\n'
            '  3) Use "2D Pose Estimate" to initialize AMCL pose\n'
    )

    return LaunchDescription(
        [
            map_yaml_la,
            amcl_params_la,
            use_sim_time_la,
            imu_yaw_offset_la,
            imu_yaw_alpha_la,
            imu_linear_speed_scale_la,
            bringup_launch,
            imu_odom_fusion_node,
            map_server_node,
            amcl_node,
            lifecycle_manager_node,
            print_usage_instructions,
        ]
    )
