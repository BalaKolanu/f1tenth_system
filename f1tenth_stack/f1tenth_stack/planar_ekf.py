"""ROS-independent extended Kalman filter for planar vehicle odometry."""

import math
from typing import Optional, Sequence

import numpy as np


X = 0
Y = 1
YAW = 2
SPEED = 3
GYRO_BIAS = 4
ACCEL_BIAS = 5
STATE_SIZE = 6


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


class PlanarEkf:
    """Fuse longitudinal IMU motion with wheel speed in a planar EKF."""

    def __init__(
        self,
        *,
        initial_covariance: Optional[Sequence[float]] = None,
        gyro_noise_std: float = 0.03,
        accel_noise_std: float = 0.8,
        gyro_bias_rw_std: float = 0.002,
        accel_bias_rw_std: float = 0.05,
        covariance_floor: float = 1e-9,
    ) -> None:
        self.state = np.zeros(STATE_SIZE, dtype=np.float64)
        diagonal = initial_covariance or (1.0, 1.0, 0.25, 1.0, 0.04, 0.25)
        if len(diagonal) != STATE_SIZE:
            raise ValueError('initial_covariance must contain six diagonal values')
        self.covariance = np.diag(np.maximum(np.asarray(diagonal, dtype=float), 0.0))
        self.gyro_noise_std = max(float(gyro_noise_std), 0.0)
        self.accel_noise_std = max(float(accel_noise_std), 0.0)
        self.gyro_bias_rw_std = max(float(gyro_bias_rw_std), 0.0)
        self.accel_bias_rw_std = max(float(accel_bias_rw_std), 0.0)
        self.covariance_floor = max(float(covariance_floor), 0.0)

    def set_position(self, x: float, y: float, variance: Optional[float] = None) -> None:
        """Initialize or externally reset the planar position."""
        self.state[X] = float(x)
        self.state[Y] = float(y)
        if variance is not None:
            value = max(float(variance), self.covariance_floor)
            self.covariance[X, X] = value
            self.covariance[Y, Y] = value

    def predict(self, dt: float, gyro_z: float, accel_x: Optional[float] = None) -> bool:
        """Propagate state using yaw rate and optional longitudinal acceleration."""
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0.0:
            return False
        if not math.isfinite(gyro_z):
            return False
        use_acceleration = accel_x is not None and math.isfinite(accel_x)

        yaw = self.state[YAW]
        speed = self.state[SPEED]
        yaw_rate = float(gyro_z) - self.state[GYRO_BIAS]
        acceleration = (
            float(accel_x) - self.state[ACCEL_BIAS]
            if use_acceleration else 0.0
        )
        midpoint_yaw = yaw + 0.5 * yaw_rate * dt
        midpoint_speed = speed + 0.5 * acceleration * dt
        cos_yaw = math.cos(midpoint_yaw)
        sin_yaw = math.sin(midpoint_yaw)

        self.state[X] += midpoint_speed * cos_yaw * dt
        self.state[Y] += midpoint_speed * sin_yaw * dt
        self.state[YAW] = normalize_angle(yaw + yaw_rate * dt)
        self.state[SPEED] += acceleration * dt

        transition = np.eye(STATE_SIZE, dtype=np.float64)
        transition[X, YAW] = -midpoint_speed * sin_yaw * dt
        transition[X, SPEED] = cos_yaw * dt
        transition[X, GYRO_BIAS] = 0.5 * midpoint_speed * sin_yaw * dt * dt
        if use_acceleration:
            transition[X, ACCEL_BIAS] = -0.5 * cos_yaw * dt * dt
        transition[Y, YAW] = midpoint_speed * cos_yaw * dt
        transition[Y, SPEED] = sin_yaw * dt
        transition[Y, GYRO_BIAS] = -0.5 * midpoint_speed * cos_yaw * dt * dt
        if use_acceleration:
            transition[Y, ACCEL_BIAS] = -0.5 * sin_yaw * dt * dt
        transition[YAW, GYRO_BIAS] = -dt
        if use_acceleration:
            transition[SPEED, ACCEL_BIAS] = -dt

        noise_map = np.zeros((STATE_SIZE, 4), dtype=np.float64)
        noise_map[X, 0] = -0.5 * midpoint_speed * sin_yaw * dt * dt
        noise_map[Y, 0] = 0.5 * midpoint_speed * cos_yaw * dt * dt
        noise_map[YAW, 0] = dt
        if use_acceleration:
            noise_map[X, 1] = 0.5 * cos_yaw * dt * dt
            noise_map[Y, 1] = 0.5 * sin_yaw * dt * dt
            noise_map[SPEED, 1] = dt
        noise_map[GYRO_BIAS, 2] = math.sqrt(dt)
        noise_map[ACCEL_BIAS, 3] = math.sqrt(dt)
        noise_variance = np.diag([
            self.gyro_noise_std ** 2,
            self.accel_noise_std ** 2,
            self.gyro_bias_rw_std ** 2,
            self.accel_bias_rw_std ** 2,
        ])
        process_noise = noise_map @ noise_variance @ noise_map.T
        self.covariance = transition @ self.covariance @ transition.T + process_noise
        self._stabilize_covariance()
        return True

    def update_wheel_speed(
        self,
        speed: float,
        variance: float,
        innovation_gate_sigma: float = 6.0,
    ) -> bool:
        """Correct forward speed using the wheel-odometry velocity."""
        return self._update_scalar(
            measurement=float(speed),
            predicted=float(self.state[SPEED]),
            measurement_variance=variance,
            measurement_index=SPEED,
            innovation_gate_sigma=innovation_gate_sigma,
            angular=False,
        )

    def update_yaw(
        self,
        yaw: float,
        variance: float,
        innovation_gate_sigma: float = 6.0,
    ) -> bool:
        """Correct heading using a normalized IMU orientation measurement."""
        return self._update_scalar(
            measurement=normalize_angle(yaw),
            predicted=float(self.state[YAW]),
            measurement_variance=variance,
            measurement_index=YAW,
            innovation_gate_sigma=innovation_gate_sigma,
            angular=True,
        )

    def reset_yaw(self, yaw: float, variance: float) -> None:
        """Recover heading from a trusted orientation after persistent rejection."""
        self._reset_scalar(YAW, normalize_angle(yaw), variance)

    def reset_speed(self, speed: float, variance: float) -> None:
        """Recover speed from trusted wheel odometry after persistent rejection."""
        self._reset_scalar(SPEED, float(speed), variance)

    def _reset_scalar(self, index: int, value: float, variance: float) -> None:
        self.state[index] = value
        self.covariance[index, :] = 0.0
        self.covariance[:, index] = 0.0
        self.covariance[index, index] = max(
            float(variance),
            self.covariance_floor,
        )
        self._stabilize_covariance()

    def _update_scalar(
        self,
        *,
        measurement: float,
        predicted: float,
        measurement_variance: float,
        measurement_index: int,
        innovation_gate_sigma: float,
        angular: bool,
    ) -> bool:
        if not math.isfinite(measurement):
            return False
        variance = max(float(measurement_variance), self.covariance_floor)
        if not math.isfinite(variance):
            return False

        innovation = measurement - predicted
        if angular:
            innovation = normalize_angle(innovation)
        innovation_variance = float(
            self.covariance[measurement_index, measurement_index] + variance
        )
        if innovation_variance <= 0.0 or not math.isfinite(innovation_variance):
            return False
        gate = max(float(innovation_gate_sigma), 0.0)
        if gate > 0.0 and innovation * innovation > gate * gate * innovation_variance:
            return False

        observation = np.zeros((1, STATE_SIZE), dtype=np.float64)
        observation[0, measurement_index] = 1.0
        gain = self.covariance @ observation.T / innovation_variance
        self.state += gain[:, 0] * innovation
        self.state[YAW] = normalize_angle(self.state[YAW])

        identity = np.eye(STATE_SIZE, dtype=np.float64)
        residual_map = identity - gain @ observation
        self.covariance = (
            residual_map @ self.covariance @ residual_map.T
            + gain * variance @ gain.T
        )
        self._stabilize_covariance()
        return True

    def _stabilize_covariance(self) -> None:
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        diagonal = np.diag(self.covariance).copy()
        diagonal = np.maximum(diagonal, self.covariance_floor)
        np.fill_diagonal(self.covariance, diagonal)
