from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="9090",
        description="WebSocket port exposed to Foxglove via rosbridge",
    )
    address_arg = DeclareLaunchArgument(
        "address",
        default_value="",
        description="Bind address for rosbridge_websocket",
    )
    retry_startup_delay_arg = DeclareLaunchArgument(
        "retry_startup_delay",
        default_value="5.0",
        description="Seconds to wait before retrying port bind on startup",
    )

    rosbridge_node = Node(
        package="f1tenth_stack",
        executable="safe_rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "address": LaunchConfiguration("address"),
                "retry_startup_delay": LaunchConfiguration("retry_startup_delay"),
            }
        ],
    )

    rosapi_node = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        output="screen",
    )

    return LaunchDescription(
        [
            port_arg,
            address_arg,
            retry_startup_delay_arg,
            rosbridge_node,
            rosapi_node,
        ]
    )
