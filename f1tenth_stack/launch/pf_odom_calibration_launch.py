from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_topic_la = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odometry/imu_fused',
        description='Odometry topic to calibrate',
    )
    reference_topic_la = DeclareLaunchArgument(
        'reference_topic',
        default_value='/pf/pose/odom',
        description='Reference localization topic',
    )
    current_linear_speed_scale_la = DeclareLaunchArgument(
        'current_linear_speed_scale',
        default_value='1.0',
        description='Current imu_odom_fusion linear_speed_scale value',
    )

    linear_odom_calibrator_node = Node(
        package='f1tenth_stack',
        executable='linear_odom_calibrator',
        name='linear_odom_calibrator',
        output='screen',
        parameters=[
            {
                'odom_topic': LaunchConfiguration('odom_topic'),
                'reference_topic': LaunchConfiguration('reference_topic'),
                'reference_topic_type': 'odometry',
                'sampling_frequency': 20.0,
                'max_yaw_rate': 0.20,
                'max_pair_jump': 0.35,
                'min_pair_step': 0.002,
                'min_reference_distance': 5.0,
                'current_linear_speed_scale': LaunchConfiguration('current_linear_speed_scale'),
            },
        ],
    )

    usage = LogInfo(
        msg='PF odom calibration started.\n'
            'Run this alongside your usual localization stack.\n'
            'It compares /odometry/imu_fused against /pf/pose/odom and prints\n'
            'a recommended linear_speed_scale on shutdown.\n'
            'Drive mostly straight when collecting calibration data.\n'
    )

    return LaunchDescription(
        [
            odom_topic_la,
            reference_topic_la,
            current_linear_speed_scale_la,
            linear_odom_calibrator_node,
            usage,
        ]
    )
