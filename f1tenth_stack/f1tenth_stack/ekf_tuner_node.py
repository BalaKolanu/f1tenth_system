#!/usr/bin/env python3

"""Estimate EKF tuning values from particle-filter localization motion."""

# Copyright (c) 2026 Purdue University
# SPDX-License-Identifier: MIT

import json
import math
from typing import Optional, Tuple

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu
from std_msgs.msg import String


Pose2d = Tuple[float, float, float]


def _normalize_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_from_orientation(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def _stamp_sec(message) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


class ExponentialStats:
    """Track a slowly varying mean, variance, and RMS without unbounded storage."""

    def __init__(self, alpha: float) -> None:
        self.alpha = _clamp(alpha, 1e-4, 1.0)
        self.count = 0
        self.mean = 0.0
        self.variance = 0.0
        self.mean_square = 0.0

    def update(self, value: float) -> None:
        if not math.isfinite(value):
            return
        if self.count == 0:
            self.mean = value
            self.mean_square = value * value
            self.variance = 0.0
        else:
            difference = value - self.mean
            self.mean += self.alpha * difference
            self.variance = (1.0 - self.alpha) * (
                self.variance + self.alpha * difference * difference
            )
            self.mean_square += self.alpha * (value * value - self.mean_square)
        self.count += 1

    @property
    def standard_deviation(self) -> float:
        return math.sqrt(max(self.variance, 0.0))

    @property
    def rms(self) -> float:
        return math.sqrt(max(self.mean_square, 0.0))


class EkfTunerNode(Node):
    """Compare EKF and sensor motion against independent PF localization."""

    def __init__(self) -> None:
        super().__init__('ekf_tuner_node')
        self._declare_parameters()
        self.max_sensor_age = float(self.get_parameter('max_sensor_age_sec').value)
        self.min_dt = float(self.get_parameter('min_reference_dt_sec').value)
        self.max_dt = float(self.get_parameter('max_reference_dt_sec').value)
        self.min_wheel_speed = float(self.get_parameter('min_wheel_speed_mps').value)
        self.max_speed = float(self.get_parameter('max_reference_speed_mps').value)
        self.max_yaw_rate = float(
            self.get_parameter('max_reference_yaw_rate_rps').value
        )
        self.minimum_samples = int(self.get_parameter('minimum_samples').value)
        alpha = float(self.get_parameter('estimator_alpha').value)

        self.gyro_bias = ExponentialStats(alpha)
        self.accel_bias = ExponentialStats(alpha)
        self.wheel_scale = ExponentialStats(alpha)
        self.gyro_residual = ExponentialStats(alpha)
        self.accel_residual = ExponentialStats(alpha)
        self.wheel_residual = ExponentialStats(alpha)
        self.imu_yaw_residual = ExponentialStats(alpha)
        self.ekf_position_error = ExponentialStats(alpha)
        self.ekf_yaw_error = ExponentialStats(alpha)

        self.latest_imu = None
        self.latest_wheel = None
        self.latest_ekf = None
        self.previous_pf = None
        self.previous_reference_speed: Optional[float] = None
        self.reference_yaw_origin: Optional[float] = None
        self.imu_yaw_origin: Optional[float] = None
        self.map_from_ekf = None
        self.accepted_reference_updates = 0
        self.rejected_reference_updates = 0

        pf_topic = str(self.get_parameter('pf_odom_topic').value)
        ekf_topic = str(self.get_parameter('ekf_odom_topic').value)
        wheel_topic = str(self.get_parameter('wheel_odom_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        report_topic = str(self.get_parameter('report_topic').value)
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(Imu, imu_topic, self._handle_imu, sensor_qos)
        self.create_subscription(Odometry, wheel_topic, self._handle_wheel, 50)
        self.create_subscription(Odometry, ekf_topic, self._handle_ekf, 50)
        self.create_subscription(Odometry, pf_topic, self._handle_pf, 10)
        self.report_publisher = self.create_publisher(String, report_topic, 10)
        report_period = max(float(self.get_parameter('report_period_sec').value), 0.2)
        self.create_timer(report_period, self._publish_report)
        self.get_logger().info(
            'EKF tuner ready: reference=%s ekf=%s wheel=%s imu=%s report=%s'
            % (pf_topic, ekf_topic, wheel_topic, imu_topic, report_topic)
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('pf_odom_topic', '/pf/pose/odom')
        self.declare_parameter('ekf_odom_topic', '/odometry/ekf')
        self.declare_parameter('wheel_odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/sensors/imu/raw')
        self.declare_parameter('report_topic', '/diagnostics/ekf_tuning')
        self.declare_parameter('report_period_sec', 2.0)
        self.declare_parameter('max_sensor_age_sec', 0.15)
        self.declare_parameter('min_reference_dt_sec', 0.03)
        self.declare_parameter('max_reference_dt_sec', 0.5)
        self.declare_parameter('min_wheel_speed_mps', 0.5)
        self.declare_parameter('max_reference_speed_mps', 15.0)
        self.declare_parameter('max_reference_yaw_rate_rps', 8.0)
        self.declare_parameter('estimator_alpha', 0.03)
        self.declare_parameter('minimum_samples', 50)

    @staticmethod
    def _pose(message: Odometry) -> Pose2d:
        return (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            _yaw_from_orientation(message.pose.pose.orientation),
        )

    def _handle_imu(self, message: Imu) -> None:
        yaw = _yaw_from_orientation(message.orientation)
        self.latest_imu = (
            _stamp_sec(message),
            float(message.angular_velocity.z),
            float(message.linear_acceleration.x),
            yaw,
        )

    def _handle_wheel(self, message: Odometry) -> None:
        self.latest_wheel = (
            _stamp_sec(message),
            float(message.twist.twist.linear.x),
        )

    def _handle_ekf(self, message: Odometry) -> None:
        self.latest_ekf = (_stamp_sec(message), self._pose(message))

    def _fresh(self, sample, reference_stamp: float) -> bool:
        return sample is not None and abs(sample[0] - reference_stamp) <= self.max_sensor_age

    def _handle_pf(self, message: Odometry) -> None:
        stamp = _stamp_sec(message)
        pose = self._pose(message)
        self._update_ekf_error(stamp, pose)
        if self.previous_pf is None:
            self.previous_pf = (stamp, pose)
            self._set_yaw_origins(pose)
            return

        previous_stamp, previous_pose = self.previous_pf
        dt = stamp - previous_stamp
        self.previous_pf = (stamp, pose)
        if dt < self.min_dt or dt > self.max_dt:
            self.previous_reference_speed = None
            self.rejected_reference_updates += 1
            return

        yaw_delta = _normalize_angle(pose[2] - previous_pose[2])
        midpoint_yaw = previous_pose[2] + 0.5 * yaw_delta
        dx = pose[0] - previous_pose[0]
        dy = pose[1] - previous_pose[1]
        forward_distance = math.cos(midpoint_yaw) * dx + math.sin(midpoint_yaw) * dy
        reference_speed = forward_distance / dt
        reference_yaw_rate = yaw_delta / dt
        if abs(reference_speed) > self.max_speed or abs(reference_yaw_rate) > self.max_yaw_rate:
            self.previous_reference_speed = None
            self.rejected_reference_updates += 1
            return

        reference_acceleration = None
        if self.previous_reference_speed is not None:
            reference_acceleration = (reference_speed - self.previous_reference_speed) / dt
        self.previous_reference_speed = reference_speed
        self.accepted_reference_updates += 1
        self._update_imu_estimates(
            stamp,
            pose,
            reference_yaw_rate,
            reference_acceleration,
        )
        self._update_wheel_estimates(stamp, reference_speed)

    def _set_yaw_origins(self, pf_pose: Pose2d) -> None:
        if self.reference_yaw_origin is None and self.latest_imu is not None:
            self.reference_yaw_origin = pf_pose[2]
            self.imu_yaw_origin = self.latest_imu[3]

    def _update_imu_estimates(
        self,
        stamp: float,
        pf_pose: Pose2d,
        reference_yaw_rate: float,
        reference_acceleration: Optional[float],
    ) -> None:
        if not self._fresh(self.latest_imu, stamp):
            return
        _, gyro_z, accel_x, imu_yaw = self.latest_imu
        gyro_bias_sample = gyro_z - reference_yaw_rate
        if abs(gyro_bias_sample) < 2.0:
            self.gyro_bias.update(gyro_bias_sample)
            self.gyro_residual.update(gyro_bias_sample - self.gyro_bias.mean)

        if reference_acceleration is not None:
            accel_bias_sample = accel_x - reference_acceleration
            if abs(accel_bias_sample) < 15.0:
                self.accel_bias.update(accel_bias_sample)
                self.accel_residual.update(accel_bias_sample - self.accel_bias.mean)

        self._set_yaw_origins(pf_pose)
        if self.reference_yaw_origin is not None and self.imu_yaw_origin is not None:
            reference_relative_yaw = _normalize_angle(
                pf_pose[2] - self.reference_yaw_origin
            )
            imu_relative_yaw = _normalize_angle(imu_yaw - self.imu_yaw_origin)
            self.imu_yaw_residual.update(
                _normalize_angle(imu_relative_yaw - reference_relative_yaw)
            )

    def _update_wheel_estimates(self, stamp: float, reference_speed: float) -> None:
        if not self._fresh(self.latest_wheel, stamp):
            return
        wheel_speed = self.latest_wheel[1]
        if abs(wheel_speed) >= self.min_wheel_speed:
            scale_sample = reference_speed / wheel_speed
            if 0.25 <= scale_sample <= 4.0:
                self.wheel_scale.update(scale_sample)
        scale = self.wheel_scale.mean if self.wheel_scale.count else 1.0
        self.wheel_residual.update(reference_speed - scale * wheel_speed)

    def _update_ekf_error(self, stamp: float, pf_pose: Pose2d) -> None:
        if not self._fresh(self.latest_ekf, stamp):
            return
        ekf_pose = self.latest_ekf[1]
        if self.map_from_ekf is None:
            angle = _normalize_angle(pf_pose[2] - ekf_pose[2])
            cos_angle = math.cos(angle)
            sin_angle = math.sin(angle)
            translated_x = pf_pose[0] - (cos_angle * ekf_pose[0] - sin_angle * ekf_pose[1])
            translated_y = pf_pose[1] - (sin_angle * ekf_pose[0] + cos_angle * ekf_pose[1])
            self.map_from_ekf = (translated_x, translated_y, angle)
            return
        translation_x, translation_y, angle = self.map_from_ekf
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        aligned_x = translation_x + cos_angle * ekf_pose[0] - sin_angle * ekf_pose[1]
        aligned_y = translation_y + sin_angle * ekf_pose[0] + cos_angle * ekf_pose[1]
        position_error = math.hypot(aligned_x - pf_pose[0], aligned_y - pf_pose[1])
        yaw_error = _normalize_angle(ekf_pose[2] + angle - pf_pose[2])
        if math.isfinite(position_error):
            self.ekf_position_error.update(position_error)
        if abs(yaw_error) < math.pi:
            self.ekf_yaw_error.update(yaw_error)

    def _publish_report(self) -> None:
        sample_count = min(
            self.gyro_bias.count,
            self.wheel_scale.count,
            self.imu_yaw_residual.count,
        )
        ready = sample_count >= self.minimum_samples
        recommendations = {
            'initial_gyro_bias': self.gyro_bias.mean,
            'initial_accel_bias': self.accel_bias.mean,
            'wheel_speed_scale': self.wheel_scale.mean if self.wheel_scale.count else 1.0,
            'gyro_noise_std': _clamp(self.gyro_residual.standard_deviation, 0.005, 0.5),
            'accel_noise_std': _clamp(self.accel_residual.standard_deviation, 0.1, 5.0),
            'wheel_speed_std': _clamp(self.wheel_residual.standard_deviation, 0.02, 2.0),
            'imu_yaw_std': _clamp(self.imu_yaw_residual.standard_deviation, 0.02, 0.8),
        }
        report = {
            'ready': ready,
            'accepted_reference_updates': self.accepted_reference_updates,
            'rejected_reference_updates': self.rejected_reference_updates,
            'samples': {
                'gyro': self.gyro_bias.count,
                'accel': self.accel_bias.count,
                'wheel_scale': self.wheel_scale.count,
                'ekf_error': self.ekf_position_error.count,
            },
            'ekf_error': {
                'position_rmse_m': self.ekf_position_error.rms,
                'yaw_rmse_rad': self.ekf_yaw_error.rms,
            },
            'recommended_ekf_parameters': recommendations,
        }
        message = String()
        message.data = json.dumps(report, sort_keys=True)
        self.report_publisher.publish(message)
        if ready:
            self.get_logger().info('EKF tuning recommendation: %s' % message.data)


def main(args=None) -> None:
    """Run the PF-referenced EKF tuner."""
    rclpy.init(args=args)
    node = EkfTunerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
