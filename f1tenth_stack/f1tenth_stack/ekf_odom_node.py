#!/usr/bin/env python3

"""ROS 2 wrapper for planar wheel-odometry and IMU EKF fusion."""

# Copyright (c) 2026 Purdue University
# SPDX-License-Identifier: MIT

import math
import time
from typing import Optional, Tuple

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from f1tenth_stack.planar_ekf import (
    ACCEL_BIAS,
    GYRO_BIAS,
    SPEED,
    X,
    Y,
    YAW,
    PlanarEkf,
    normalize_angle,
)


Quaternion = Tuple[float, float, float, float]
Vector3 = Tuple[float, float, float]


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _normalize_quaternion(quaternion: Quaternion) -> Optional[Quaternion]:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-9:
        return None
    return (x / norm, y / norm, z / norm, w / norm)


def _quaternion_conjugate(quaternion: Quaternion) -> Quaternion:
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def _quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
    rotated = _quaternion_multiply(
        _quaternion_multiply(quaternion, vector_quaternion),
        _quaternion_conjugate(quaternion),
    )
    return (rotated[0], rotated[1], rotated[2])


def _yaw_from_quaternion(quaternion: Quaternion) -> float:
    x, y, z, w = quaternion
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _yaw_quaternion(yaw: float) -> Quaternion:
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def _positive_variance(values, index: int, fallback: float) -> float:
    if len(values) > index:
        value = float(values[index])
        if math.isfinite(value) and value > 0.0:
            return value
    return fallback


class EkfOdomNode(Node):
    """Estimate planar odometry from a body-frame IMU and wheel speed."""

    def __init__(self) -> None:
        super().__init__('ekf_odom_node')
        self._declare_parameters()
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.wheel_odom_topic = str(self.get_parameter('wheel_odom_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = _as_bool(self.get_parameter('publish_tf').value)
        self.use_imu_orientation = _as_bool(
            self.get_parameter('use_imu_orientation').value
        )
        self.use_accel_prediction = _as_bool(
            self.get_parameter('use_accel_prediction').value
        )
        self.zero_initial_yaw = _as_bool(self.get_parameter('zero_initial_yaw').value)
        self.initialize_position_from_wheel_odom = _as_bool(
            self.get_parameter('initialize_position_from_wheel_odom').value
        )
        self.wheel_speed_scale = float(self.get_parameter('wheel_speed_scale').value)
        self.max_dt_sec = max(float(self.get_parameter('max_dt_sec').value), 1e-3)
        self.wheel_speed_variance = self._variance_parameter('wheel_speed_std')
        self.imu_yaw_variance = self._variance_parameter('imu_yaw_std')
        self.gyro_variance = self._variance_parameter('gyro_noise_std')
        self.wheel_gate = float(self.get_parameter('wheel_innovation_gate_sigma').value)
        self.yaw_gate = float(self.get_parameter('yaw_innovation_gate_sigma').value)
        self.recovery_rejections = max(
            int(self.get_parameter('measurement_recovery_rejections').value),
            1,
        )
        initial_covariance = list(self.get_parameter('initial_covariance').value)

        self.filter = PlanarEkf(
            initial_covariance=initial_covariance,
            gyro_noise_std=float(self.get_parameter('gyro_noise_std').value),
            accel_noise_std=float(self.get_parameter('accel_noise_std').value),
            gyro_bias_rw_std=float(self.get_parameter('gyro_bias_rw_std').value),
            accel_bias_rw_std=float(self.get_parameter('accel_bias_rw_std').value),
        )
        self.filter.state[GYRO_BIAS] = float(
            self.get_parameter('initial_gyro_bias').value
        )
        self.filter.state[ACCEL_BIAS] = float(
            self.get_parameter('initial_accel_bias').value
        )
        self.last_filter_time_sec: Optional[float] = None
        self.latest_gyro_z = 0.0
        self.latest_accel_x = 0.0
        self.initial_yaw_offset: Optional[float] = None
        self.position_initialized = False
        self.rejected_wheel_updates = 0
        self.rejected_yaw_updates = 0
        self.consecutive_wheel_rejections = 0
        self.consecutive_yaw_rejections = 0
        self.last_warning_times = {}
        self.imu_rotation_cache = {}

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, self.output_topic, 20)
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(Imu, self.imu_topic, self._handle_imu, sensor_qos)
        self.create_subscription(
            Odometry,
            self.wheel_odom_topic,
            self._handle_wheel_odom,
            50,
        )
        self.get_logger().info(
            'Planar EKF ready: imu=%s wheel_odom=%s output=%s publish_tf=%s'
            % (
                self.imu_topic,
                self.wheel_odom_topic,
                self.output_topic,
                str(self.publish_tf),
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('imu_topic', '/sensors/imu/raw')
        self.declare_parameter('wheel_odom_topic', '/odom')
        self.declare_parameter('output_topic', '/odometry/ekf')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('use_imu_orientation', True)
        self.declare_parameter('use_accel_prediction', False)
        self.declare_parameter('zero_initial_yaw', True)
        self.declare_parameter('initialize_position_from_wheel_odom', True)
        self.declare_parameter('wheel_speed_scale', 1.0)
        self.declare_parameter('max_dt_sec', 0.1)
        self.declare_parameter('gyro_noise_std', 0.03)
        self.declare_parameter('accel_noise_std', 0.8)
        self.declare_parameter('gyro_bias_rw_std', 0.002)
        self.declare_parameter('accel_bias_rw_std', 0.05)
        self.declare_parameter('initial_gyro_bias', 0.0)
        self.declare_parameter('initial_accel_bias', 0.0)
        self.declare_parameter('wheel_speed_std', 0.15)
        self.declare_parameter('imu_yaw_std', 0.08)
        self.declare_parameter('wheel_innovation_gate_sigma', 6.0)
        self.declare_parameter('yaw_innovation_gate_sigma', 6.0)
        self.declare_parameter('measurement_recovery_rejections', 20)
        self.declare_parameter(
            'initial_covariance',
            [1.0, 1.0, 0.25, 1.0, 0.04, 0.25],
        )

    def _variance_parameter(self, name: str) -> float:
        standard_deviation = max(float(self.get_parameter(name).value), 1e-6)
        return standard_deviation * standard_deviation

    def _message_time_sec(self, message) -> float:
        stamp = message.header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if value <= 0.0:
            return self.get_clock().now().nanoseconds * 1e-9
        return value

    def _predict_to(self, target_time_sec: float) -> None:
        if self.last_filter_time_sec is None:
            self.last_filter_time_sec = target_time_sec
            return
        dt = target_time_sec - self.last_filter_time_sec
        if dt <= 0.0:
            return
        if dt > self.max_dt_sec:
            self._warn_throttled(
                'large_dt',
                'EKF input gap %.3fs exceeds max_dt_sec; limiting propagation' % dt,
            )
            dt = self.max_dt_sec
        accel_x = self.latest_accel_x if self.use_accel_prediction else None
        self.filter.predict(dt, self.latest_gyro_z, accel_x)
        self.last_filter_time_sec = target_time_sec

    def _imu_to_base_rotation(self, source_frame: str) -> Optional[Quaternion]:
        if not source_frame or source_frame == self.base_frame:
            return (0.0, 0.0, 0.0, 1.0)
        if source_frame in self.imu_rotation_cache:
            return self.imu_rotation_cache[source_frame]
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as error:
            self._warn_throttled(
                'imu_transform_' + source_frame,
                'Waiting for IMU transform %s -> %s: %s'
                % (source_frame, self.base_frame, str(error)),
            )
            return None
        rotation = transform.transform.rotation
        quaternion = _normalize_quaternion((rotation.x, rotation.y, rotation.z, rotation.w))
        if quaternion is not None:
            self.imu_rotation_cache[source_frame] = quaternion
        return quaternion

    def _handle_imu(self, message: Imu) -> None:
        base_from_imu = self._imu_to_base_rotation(message.header.frame_id)
        if base_from_imu is None:
            return
        angular_velocity = _rotate_vector(
            base_from_imu,
            (
                float(message.angular_velocity.x),
                float(message.angular_velocity.y),
                float(message.angular_velocity.z),
            ),
        )
        linear_acceleration = _rotate_vector(
            base_from_imu,
            (
                float(message.linear_acceleration.x),
                float(message.linear_acceleration.y),
                float(message.linear_acceleration.z),
            ),
        )
        if not math.isfinite(angular_velocity[2]) or not math.isfinite(
            linear_acceleration[0]
        ):
            self._warn_throttled('invalid_imu_motion', 'Ignoring non-finite IMU motion data')
            return

        stamp_sec = self._message_time_sec(message)
        self._predict_to(stamp_sec)
        self.latest_gyro_z = angular_velocity[2]
        self.latest_accel_x = linear_acceleration[0]

        if self.use_imu_orientation and message.orientation_covariance[0] >= 0.0:
            world_from_imu = _normalize_quaternion(
                (
                    float(message.orientation.x),
                    float(message.orientation.y),
                    float(message.orientation.z),
                    float(message.orientation.w),
                )
            )
            if world_from_imu is not None:
                world_from_base = _quaternion_multiply(
                    world_from_imu,
                    _quaternion_conjugate(base_from_imu),
                )
                measured_yaw = _yaw_from_quaternion(world_from_base)
                if self.initial_yaw_offset is None:
                    self.initial_yaw_offset = -measured_yaw if self.zero_initial_yaw else 0.0
                measured_yaw = normalize_angle(measured_yaw + self.initial_yaw_offset)
                yaw_variance = _positive_variance(
                    message.orientation_covariance,
                    8,
                    self.imu_yaw_variance,
                )
                if not self.filter.update_yaw(measured_yaw, yaw_variance, self.yaw_gate):
                    self.rejected_yaw_updates += 1
                    self.consecutive_yaw_rejections += 1
                    self._warn_throttled(
                        'yaw_gate',
                        'Rejected IMU yaw innovation (%d total)'
                        % self.rejected_yaw_updates,
                    )
                    if self.consecutive_yaw_rejections >= self.recovery_rejections:
                        self.filter.reset_yaw(measured_yaw, yaw_variance)
                        self.consecutive_yaw_rejections = 0
                        self.get_logger().warning(
                            'Recovered EKF heading from persistent IMU yaw innovations'
                        )
                else:
                    self.consecutive_yaw_rejections = 0
        self._publish(stamp_sec)

    def _handle_wheel_odom(self, message: Odometry) -> None:
        stamp_sec = self._message_time_sec(message)
        self._predict_to(stamp_sec)
        if self.initialize_position_from_wheel_odom and not self.position_initialized:
            position_variance = _positive_variance(
                message.pose.covariance,
                0,
                1.0,
            )
            self.filter.set_position(
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                position_variance,
            )
            self.position_initialized = True
        speed = float(message.twist.twist.linear.x) * self.wheel_speed_scale
        speed_variance = _positive_variance(
            message.twist.covariance,
            0,
            self.wheel_speed_variance,
        )
        if not self.filter.update_wheel_speed(speed, speed_variance, self.wheel_gate):
            self.rejected_wheel_updates += 1
            self.consecutive_wheel_rejections += 1
            self._warn_throttled(
                'wheel_gate',
                'Rejected wheel-speed innovation (%d total)'
                % self.rejected_wheel_updates,
            )
            if self.consecutive_wheel_rejections >= self.recovery_rejections:
                self.filter.reset_speed(speed, speed_variance)
                self.consecutive_wheel_rejections = 0
                self.get_logger().warning(
                    'Recovered EKF speed from persistent wheel-speed innovations'
                )
        else:
            self.consecutive_wheel_rejections = 0
        self._publish(stamp_sec)

    def _publish(self, stamp_sec: float) -> None:
        state = self.filter.state
        covariance = self.filter.covariance
        message = Odometry()
        stamp_nanoseconds = max(int(stamp_sec * 1e9), 0)
        message.header.stamp = Time(nanoseconds=stamp_nanoseconds).to_msg()
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = float(state[X])
        message.pose.pose.position.y = float(state[Y])
        quaternion = _yaw_quaternion(float(state[YAW]))
        message.pose.pose.orientation.x = quaternion[0]
        message.pose.pose.orientation.y = quaternion[1]
        message.pose.pose.orientation.z = quaternion[2]
        message.pose.pose.orientation.w = quaternion[3]
        message.twist.twist.linear.x = float(state[SPEED])
        message.twist.twist.angular.z = float(
            self.latest_gyro_z - state[GYRO_BIAS]
        )

        pose_mapping = ((0, X), (1, Y), (5, YAW))
        for ros_row, state_row in pose_mapping:
            for ros_column, state_column in pose_mapping:
                message.pose.covariance[ros_row * 6 + ros_column] = float(
                    covariance[state_row, state_column]
                )
        message.pose.covariance[14] = 1e6
        message.pose.covariance[21] = 1e6
        message.pose.covariance[28] = 1e6
        message.twist.covariance[0] = float(covariance[SPEED, SPEED])
        message.twist.covariance[35] = float(
            covariance[GYRO_BIAS, GYRO_BIAS] + self.gyro_variance
        )
        message.twist.covariance[7] = 1e6
        message.twist.covariance[14] = 1e6
        message.twist.covariance[21] = 1e6
        message.twist.covariance[28] = 1e6
        self.odom_publisher.publish(message)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = float(state[X])
            transform.transform.translation.y = float(state[Y])
            transform.transform.rotation = message.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def _warn_throttled(self, key: str, message: str, period_sec: float = 2.0) -> None:
        now = time.monotonic()
        previous = self.last_warning_times.get(key)
        if previous is None or now - previous >= period_sec:
            self.last_warning_times[key] = now
            self.get_logger().warning(message)


def main(args=None) -> None:
    """Run the planar EKF odometry node."""
    rclpy.init(args=args)
    node = EkfOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
