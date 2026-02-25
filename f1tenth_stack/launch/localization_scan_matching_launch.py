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
    vesc_linear_odom_config = os.path.join(
        f1tenth_share,
        'config',
        'vesc_linear_odom.yaml',
    )
    slam_localization_config = os.path.join(
        f1tenth_share,
        'config',
        'slam_toolbox_localization_scanmatching.yaml',
    )

    default_map_yaml = '/home/nvidia/spring26_ws/maps/apt_floor_2.yaml'
    default_posegraph_file = '/home/nvidia/spring26_ws/maps/apt_floor_2.posegraph'

    map_yaml_la = DeclareLaunchArgument(
        'map_yaml',
        default_value=default_map_yaml,
        description='Occupancy-grid map yaml used by map_server for visualization',
    )
    posegraph_file_la = DeclareLaunchArgument(
        'posegraph_file',
        default_value=default_posegraph_file,
        description='Serialized slam_toolbox posegraph file for localization',
    )
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock',
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
            'vesc_config': vesc_linear_odom_config,
        }.items(),
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': LaunchConfiguration('map_yaml'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': ['map_server'],
            }
        ],
    )

    joy_gated_localization_node = Node(
        package='f1tenth_stack',
        executable='joy_gated_localization',
        name='joy_gated_localization',
        output='screen',
        parameters=[
            {
                'joy_topic': '/joy',
                'enable_button_index': 0,
                'joy_timeout_sec': 0.5,
                'input_scan_topic': '/scan',
                'output_scan_topic': '/scan_localization',
                'input_pose_topic': '/scanmatching_localization/pose_raw',
                'output_pose_topic': '/scanmatching_localization/pose',
                'enabled_topic': '/scanmatching_localization/enabled',
            }
        ],
    )

    slam_localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_localization_node',
        output='screen',
        parameters=[
            slam_localization_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_file_name': LaunchConfiguration('posegraph_file'),
                'map_frame': 'map',
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'scan_topic': '/scan_localization',
                'transform_publish_period': 0.02,
            },
        ],
        remappings=[
            ('pose', '/scanmatching_localization/pose_raw'),
        ],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    print_usage_instructions = LogInfo(
        msg='Localization mode ready.\n'
            'Hold joystick button index 0 to enable scan matching updates.\n'
            'Pose topic: /scanmatching_localization/pose\n'
            'Enabled state: /scanmatching_localization/enabled\n'
            'Map topic (map_server): /map\n'
    )

    return LaunchDescription(
        [
            map_yaml_la,
            posegraph_file_la,
            use_sim_time_la,
            bringup_launch,
            map_server_node,
            map_lifecycle_manager,
            joy_gated_localization_node,
            slam_localization_node,
            print_usage_instructions,
        ]
    )
