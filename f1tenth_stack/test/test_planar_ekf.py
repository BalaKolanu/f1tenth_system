"""Deterministic tests for the ROS-independent planar EKF."""

# Copyright (c) 2026 Purdue University
# SPDX-License-Identifier: MIT

import math

import numpy as np

from f1tenth_stack.planar_ekf import ACCEL_BIAS, GYRO_BIAS, SPEED, X, Y, YAW
from f1tenth_stack.planar_ekf import PlanarEkf, normalize_angle


def test_straight_line_wheel_updates_bound_acceleration_bias():
    ekf = PlanarEkf()
    for _ in range(200):
        ekf.predict(0.01, gyro_z=0.0, accel_x=0.2)
        ekf.update_wheel_speed(2.0, variance=0.01)

    assert abs(ekf.state[SPEED] - 2.0) < 0.03
    assert abs(ekf.state[Y]) < 1e-6
    assert ekf.state[X] > 3.5
    assert ekf.state[ACCEL_BIAS] > 0.05


def test_yaw_updates_estimate_gyro_bias_and_handle_wraparound():
    ekf = PlanarEkf(initial_covariance=(0.1, 0.1, 0.2, 0.1, 0.2, 0.1))
    for _ in range(400):
        ekf.predict(0.01, gyro_z=0.05, accel_x=0.0)
        ekf.update_yaw(0.0, variance=0.0025)

    assert abs(ekf.state[YAW]) < 0.01
    assert abs(ekf.state[GYRO_BIAS] - 0.05) < 0.01

    ekf.state[YAW] = math.pi - 0.01
    assert ekf.update_yaw(-math.pi + 0.01, variance=0.01)
    assert abs(normalize_angle(ekf.state[YAW] - math.pi)) < 0.03


def test_outlier_gate_rejects_impossible_wheel_speed_jump():
    ekf = PlanarEkf(initial_covariance=(0.1,) * 6)
    previous_state = ekf.state.copy()
    previous_covariance = ekf.covariance.copy()

    accepted = ekf.update_wheel_speed(100.0, variance=0.01, innovation_gate_sigma=5.0)

    assert not accepted
    np.testing.assert_allclose(ekf.state, previous_state)
    np.testing.assert_allclose(ekf.covariance, previous_covariance)


def test_wheel_only_mode_ignores_acceleration_and_supports_recovery():
    ekf = PlanarEkf(initial_covariance=(0.1,) * 6)
    ekf.state[SPEED] = 2.0

    for _ in range(100):
        ekf.predict(0.01, gyro_z=0.0, accel_x=None)

    assert abs(ekf.state[SPEED] - 2.0) < 1e-9
    ekf.reset_speed(3.0, variance=0.04)
    ekf.reset_yaw(1.0, variance=0.01)
    assert ekf.state[SPEED] == 3.0
    assert ekf.state[YAW] == 1.0
    assert ekf.covariance[SPEED, SPEED] == 0.04
    assert ekf.covariance[YAW, YAW] == 0.01
