# f1tenth_system

## Purpose

ROS 2 Humble vehicle bringup for an F1TENTH car. This repository connects the
joystick, Ackermann command multiplexer, VESC motor controller, odometry, and
LiDAR into the standard vehicle topics used by the rest of the workspace.

The current platform uses:

- VESC speed and steering feedback
- Kinematic bicycle-model odometry from VESC speed and steering
- Ethernet Hokuyo LiDAR through urg_node
- Joystick teleoperation and Ackermann command multiplexing

## Use

From the workspace root:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
~~~

For a SICK Ethernet LiDAR:

~~~bash
ros2 launch f1tenth_stack sick_bringup_launch.py
~~~

Initialize nested repositories when cloning:

~~~bash
git submodule update --init --recursive
~~~

## Configuration

Main configuration files are under f1tenth_stack/config:

- sensors.yaml: LiDAR IP, port, frame, scan limits, and serial fallback
- vesc.yaml: VESC port, ERPM conversion, steering calibration, wheelbase, and odometry TF
- joy_teleop.yaml: joystick axes, scaling, and deadman buttons
- mux.yaml: command topics, priorities, and timeouts

The current calibration values in vesc.yaml are vehicle-specific. Validate them
before operating a different car.

The default Ethernet Hokuyo configuration expects the LiDAR address configured
in sensors.yaml. The static laser transform is defined in the bringup launch.

## Interfaces

Important outputs:

- /scan: sensor_msgs/LaserScan
- /odom: nav_msgs/Odometry
- /sensors/core: VESC telemetry
- /sensors/imu/raw: VESC IMU data
- /ackermann_drive: selected vehicle command

Important inputs:

- /teleop: joystick Ackermann commands
- /drive: autonomous Ackermann commands

## Theory and further reading

The odometry uses the bicycle-model relationship between forward velocity,
steering angle, and yaw rate. See the
[F1TENTH vehicle model documentation](https://f1tenth.github.io/build-old/ros.html)
and the [ROS 2 launch documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html).
