from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="8765",
        description="WebSocket port for the Foxglove Bridge local server",
    )
    address_arg = DeclareLaunchArgument(
        "address",
        default_value="127.0.0.1",
        description="Bind address for the Foxglove Bridge local server",
    )
    remote_access_arg = DeclareLaunchArgument(
        "remote_access",
        default_value="false",
        description="Enable Foxglove Remote Access for browser/iPad connections",
    )
    device_token_arg = DeclareLaunchArgument(
        "device_token",
        default_value=EnvironmentVariable("FOXGLOVE_DEVICE_TOKEN", default_value=""),
        description="Foxglove device token, or set FOXGLOVE_DEVICE_TOKEN",
    )
    tunnel_arg = DeclareLaunchArgument(
        "tunnel",
        default_value="false",
        description="Start a temporary Cloudflare tunnel for iPad/browser access",
    )
    cloudflared_arg = DeclareLaunchArgument(
        "cloudflared",
        default_value="/home/nvidia/.local/bin/cloudflared",
        description="Path to the cloudflared executable",
    )
    ros_domain_id_arg = DeclareLaunchArgument(
        "ros_domain_id",
        default_value=EnvironmentVariable("ROS_DOMAIN_ID", default_value="8"),
        description="ROS_DOMAIN_ID used by the car stack",
    )
    ros_localhost_only_arg = DeclareLaunchArgument(
        "ros_localhost_only",
        default_value=EnvironmentVariable("ROS_LOCALHOST_ONLY", default_value="0"),
        description="ROS_LOCALHOST_ONLY value used by the car stack",
    )

    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "address": LaunchConfiguration("address"),
                "remote_access": LaunchConfiguration("remote_access"),
                "device_token": LaunchConfiguration("device_token"),
            }
        ],
    )

    cloudflared_tunnel = ExecuteProcess(
        cmd=[
            LaunchConfiguration("cloudflared"),
            "tunnel",
            "--no-autoupdate",
            "--url",
            ["http://127.0.0.1:", LaunchConfiguration("port")],
        ],
        condition=IfCondition(LaunchConfiguration("tunnel")),
        output="screen",
    )

    return LaunchDescription(
        [
            port_arg,
            address_arg,
            remote_access_arg,
            device_token_arg,
            tunnel_arg,
            cloudflared_arg,
            ros_domain_id_arg,
            ros_localhost_only_arg,
            SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
            SetEnvironmentVariable(
                "ROS_LOCALHOST_ONLY", LaunchConfiguration("ros_localhost_only")
            ),
            foxglove_bridge_node,
            cloudflared_tunnel,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=foxglove_bridge_node,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="foxglove_bridge exited; stopping tunnel"
                            )
                        )
                    ],
                )
            ),
        ]
    )
