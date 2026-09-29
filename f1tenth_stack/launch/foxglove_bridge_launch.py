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
    max_message_size_arg = DeclareLaunchArgument(
        "max_message_size",
        default_value="10000000",
        description="Largest unfragmented rosbridge message in bytes",
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
                # /map is about 1.4 MB when sent with cbor-raw. The rosbridge
                # fragmentation path cannot serialize cbor-raw byte payloads.
                "max_message_size": LaunchConfiguration("max_message_size"),
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
            max_message_size_arg,
            rosbridge_node,
            rosapi_node,
        ]
    )
