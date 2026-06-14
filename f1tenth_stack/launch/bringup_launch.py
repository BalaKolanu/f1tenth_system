# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    joy_teleop_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'joy_teleop.yaml'
    )
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc.yaml'
    )
    sensors_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'sensors.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )
    usb_imu_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'usb_imu.yaml'
    )
    bno085_i2c_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'bno085_i2c.yaml'
    )
    imu_fusion_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'imu_odom_fusion.yaml'
    )
    hall_wheel_odom_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'hall_wheel_odom.yaml'
    )

    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs')
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')
    sensors_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=sensors_config,
        description='Descriptions for sensor configs')
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')
    usb_imu_la = DeclareLaunchArgument(
        'imu_serial_config',
        default_value=usb_imu_config,
        description='Configurations for USB serial IMU node')
    bno085_i2c_la = DeclareLaunchArgument(
        'imu_i2c_config',
        default_value=bno085_i2c_config,
        description='Configurations for BNO085 I2C IMU node')
    imu_fusion_la = DeclareLaunchArgument(
        'imu_fusion_config',
        default_value=imu_fusion_config,
        description='Configurations for IMU odom fusion node')
    hall_odom_la = DeclareLaunchArgument(
        'hall_odom_config',
        default_value=hall_wheel_odom_config,
        description='Configurations for Hall sensor wheel odom node')
    imu_frame_id_la = DeclareLaunchArgument(
        'imu_frame_id',
        default_value='imu_frame',
        description='Frame id used by the IMU publishers and static TF')
    launch_usb_imu_la = DeclareLaunchArgument(
        'launch_usb_imu',
        default_value='false',
        description='Launch USB serial IMU publisher node')
    launch_bno085_i2c_la = DeclareLaunchArgument(
        'launch_bno085_i2c',
        default_value='true',
        description='Launch BNO085 I2C IMU publisher node')
    launch_imu_fusion_la = DeclareLaunchArgument(
        'launch_imu_fusion',
        default_value='false',
        description='Launch IMU + wheel odom fusion node')
    imu_topic_la = DeclareLaunchArgument(
        'imu_topic',
        default_value='/sensors/imu/raw',
        description='IMU topic consumed by the fusion pipeline')
    imu_fused_odom_topic_la = DeclareLaunchArgument(
        'imu_fused_odom_topic',
        default_value='/odometry/imu_fused',
        description='Fused odometry topic published by the IMU fusion node')
    imu_wheel_odom_topic_la = DeclareLaunchArgument(
        'imu_wheel_odom_topic',
        default_value='/odom',
        description='Wheel odometry topic consumed by the IMU fusion node')
    imu_fusion_publish_tf_la = DeclareLaunchArgument(
        'imu_fusion_publish_tf',
        default_value='true',
        description='Publish odom->base_link TF from the IMU fusion node')
    imu_yaw_offset_la = DeclareLaunchArgument(
        'imu_yaw_offset_rad',
        default_value='0.0',
        description='Static yaw offset added to IMU heading (radians)')
    imu_yaw_alpha_la = DeclareLaunchArgument(
        'imu_yaw_alpha',
        default_value='0.3',
        description='IMU yaw smoothing factor in [0,1], higher tracks faster')
    imu_linear_speed_scale_la = DeclareLaunchArgument(
        'imu_linear_speed_scale',
        default_value='1.0',
        description='Scale factor applied to wheel odom linear speed in fusion')
    imu_wheel_speed_alpha_la = DeclareLaunchArgument(
        'imu_wheel_speed_alpha',
        default_value='0.85',
        description='Weight assigned to wheel speed versus IMU-accelerated prediction in fusion')
    launch_static_tf_la = DeclareLaunchArgument(
        'launch_static_tf',
        default_value='true',
        description='Launch static transform publishers for base_link sensor frames')
    imu_static_x_la = DeclareLaunchArgument(
        'imu_static_x',
        default_value='-0.27',
        description='Static base_link->imu_frame translation x (meters)')
    imu_static_y_la = DeclareLaunchArgument(
        'imu_static_y',
        default_value='0.0',
        description='Static base_link->imu_frame translation y (meters)')
    imu_static_z_la = DeclareLaunchArgument(
        'imu_static_z',
        default_value='0.11',
        description='Static base_link->imu_frame translation z (meters)')
    imu_static_yaw_la = DeclareLaunchArgument(
        'imu_static_yaw',
        default_value='-1.57079632679',
        description='Static base_link->imu_frame yaw (radians)')
    imu_static_pitch_la = DeclareLaunchArgument(
        'imu_static_pitch',
        default_value='0.0',
        description='Static base_link->imu_frame pitch (radians)')
    imu_static_roll_la = DeclareLaunchArgument(
        'imu_static_roll',
        default_value='0.0',
        description='Static base_link->imu_frame roll (radians)')
    vesc_to_odom_la = DeclareLaunchArgument(
        'launch_vesc_to_odom',
        default_value='false',
        description='Launch vesc_to_odom node and its odom->base_link TF')
    launch_hall_odom_la = DeclareLaunchArgument(
        'launch_hall_odom',
        default_value='true',
        description='Launch Hall sensor wheel odom node as the sole /odom publisher')
    vesc_driver_log_level_la = DeclareLaunchArgument(
        'vesc_driver_log_level',
        default_value='warn',
        description='Log level for vesc_driver_node (debug, info, warn, error, fatal)')
    vesc_publish_imu_la = DeclareLaunchArgument(
        'vesc_publish_imu',
        default_value='false',
        description='Publish onboard VESC IMU data on diagnostic-only VESC-specific topics')
    motor_speed_output_topic_la = DeclareLaunchArgument(
        'motor_speed_output_topic',
        default_value='commands/motor/speed',
        description='Output topic used by ackermann_to_vesc for motor speed commands')
    launch_tf_speed_monitor_la = DeclareLaunchArgument(
        'launch_tf_speed_monitor',
        default_value='true',
        description='Launch TF-based speed monitor node (publishes m/s and km/h)')
    tf_speed_publish_hz_la = DeclareLaunchArgument(
        'tf_speed_publish_hz',
        default_value='30.0',
        description='Publishing rate for TF speed monitor')
    tf_speed_lowpass_alpha_la = DeclareLaunchArgument(
        'tf_speed_lowpass_alpha',
        default_value='0.35',
        description='Low-pass alpha in [0,1] for TF speed monitor')
    tf_speed_source_frame_la = DeclareLaunchArgument(
        'tf_speed_source_frame',
        default_value='map',
        description='Source frame for TF speed monitor')
    tf_speed_target_frame_la = DeclareLaunchArgument(
        'tf_speed_target_frame',
        default_value='base_link',
        description='Target frame for TF speed monitor')

    ld = LaunchDescription(
        [
            joy_la, vesc_la, sensors_la, mux_la, usb_imu_la, bno085_i2c_la,
            imu_fusion_la, imu_frame_id_la, launch_usb_imu_la, launch_bno085_i2c_la, launch_imu_fusion_la,
            hall_odom_la,
            imu_topic_la, imu_fused_odom_topic_la, imu_wheel_odom_topic_la,
            imu_fusion_publish_tf_la, imu_yaw_offset_la, imu_yaw_alpha_la,
            imu_linear_speed_scale_la, imu_wheel_speed_alpha_la,
            launch_static_tf_la, imu_static_x_la, imu_static_y_la, imu_static_z_la,
            imu_static_yaw_la, imu_static_pitch_la, imu_static_roll_la,
            vesc_to_odom_la, launch_hall_odom_la, vesc_driver_log_level_la, vesc_publish_imu_la,
            motor_speed_output_topic_la, launch_tf_speed_monitor_la,
            tf_speed_publish_hz_la, tf_speed_lowpass_alpha_la,
            tf_speed_source_frame_la, tf_speed_target_frame_la
        ]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')],
        remappings=[
            ('commands/motor/speed', LaunchConfiguration('motor_speed_output_topic'))
        ],
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')],
        condition=IfCondition(LaunchConfiguration('launch_vesc_to_odom'))
    )
    hall_wheel_odom_node = Node(
        package='f1tenth_stack',
        executable='hall_wheel_odom_node',
        name='hall_wheel_odom_node',
        output='screen',
        parameters=[LaunchConfiguration('hall_odom_config')],
        condition=IfCondition(LaunchConfiguration('launch_hall_odom'))
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[
            LaunchConfiguration('vesc_config'),
            {'publish_imu': LaunchConfiguration('vesc_publish_imu')},
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            LaunchConfiguration('vesc_driver_log_level'),
        ]
    )
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[LaunchConfiguration('sensors_config')]
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser'],
        condition=IfCondition(LaunchConfiguration('launch_static_tf'))
    )
    static_imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        arguments=[
            LaunchConfiguration('imu_static_x'),
            LaunchConfiguration('imu_static_y'),
            LaunchConfiguration('imu_static_z'),
            LaunchConfiguration('imu_static_yaw'),
            LaunchConfiguration('imu_static_pitch'),
            LaunchConfiguration('imu_static_roll'),
            'base_link',
            LaunchConfiguration('imu_frame_id'),
        ],
        condition=IfCondition(LaunchConfiguration('launch_static_tf'))
    )
    usb_imu_node = Node(
        package='f1tenth_stack',
        executable='usb_imu_serial_node',
        name='usb_imu_serial_node',
        parameters=[
            LaunchConfiguration('imu_serial_config'),
            {'frame_id': LaunchConfiguration('imu_frame_id')},
        ],
        condition=IfCondition(LaunchConfiguration('launch_usb_imu'))
    )
    bno085_i2c_node = Node(
        package='f1tenth_stack',
        executable='bno085_i2c_node',
        name='bno085_i2c_node',
        parameters=[
            LaunchConfiguration('imu_i2c_config'),
            {'frame_id': LaunchConfiguration('imu_frame_id')},
        ],
        condition=IfCondition(LaunchConfiguration('launch_bno085_i2c'))
    )
    imu_odom_fusion_node = Node(
        package='f1tenth_stack',
        executable='imu_odom_fusion_node',
        name='imu_odom_fusion_node',
        output='screen',
        parameters=[
            LaunchConfiguration('imu_fusion_config'),
            {
                'imu_topic': LaunchConfiguration('imu_topic'),
                'wheel_odom_topic': LaunchConfiguration('imu_wheel_odom_topic'),
                'fused_odom_topic': LaunchConfiguration('imu_fused_odom_topic'),
                'imu_frame_id': LaunchConfiguration('imu_frame_id'),
                'publish_tf': LaunchConfiguration('imu_fusion_publish_tf'),
                'linear_speed_scale': LaunchConfiguration('imu_linear_speed_scale'),
                'yaw_offset_rad': LaunchConfiguration('imu_yaw_offset_rad'),
                'yaw_alpha': LaunchConfiguration('imu_yaw_alpha'),
                'wheel_speed_alpha': LaunchConfiguration('imu_wheel_speed_alpha'),
            },
        ],
        condition=IfCondition(LaunchConfiguration('launch_imu_fusion'))
    )
    tf_speed_monitor_node = Node(
        package='f1tenth_stack',
        executable='tf_speed_monitor',
        name='tf_speed_monitor',
        output='screen',
        parameters=[
            {
                'source_frame': LaunchConfiguration('tf_speed_source_frame'),
                'target_frame': LaunchConfiguration('tf_speed_target_frame'),
                'speed_mps_topic': '/analysis/tf_speed_mps',
                'speed_kmph_topic': '/analysis/tf_speed_kmph',
                'publish_hz': LaunchConfiguration('tf_speed_publish_hz'),
                'lowpass_alpha': LaunchConfiguration('tf_speed_lowpass_alpha'),
                'lookup_timeout_sec': 0.05,
                'max_dt_sec': 0.25,
                'max_position_jump_m': 0.75,
            }
        ],
        condition=IfCondition(LaunchConfiguration('launch_tf_speed_monitor'))
    )

    # finalize
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(hall_wheel_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(urg_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(static_tf_node)
    ld.add_action(static_imu_tf_node)
    ld.add_action(usb_imu_node)
    ld.add_action(bno085_i2c_node)
    ld.add_action(imu_odom_fusion_node)
    ld.add_action(tf_speed_monitor_node)

    return ld
