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


def generate_launch_description():
    vesc_linear_odom_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc_linear_odom.yaml',
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
            'vesc_config': vesc_linear_odom_config,
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

    print_usage_instructions = LogInfo(
        msg='Scan-matching mapping mode with linear odom enabled '
            '(steering-based odom yaw disabled).\n'
            'To save the resultant map, keep this session running, open a new '
            'terminal and run:\n'
            '\tros2 run nav2_map_server map_saver_cli -f your_map_file_name '
            '--ros-args -p map_subscribe_transient_local:=true\n'
    )

    return LaunchDescription(
        [
            bringup_launch,
            scanmatching_slam_launch,
            print_usage_instructions,
        ]
    )
