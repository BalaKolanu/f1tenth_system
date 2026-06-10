#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2026 Bala Kolanu
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import math
import os
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu, MagneticField

_IMPORT_ERROR = None


def _ensure_supported_jetson_model():
    if os.environ.get('JETSON_MODEL_NAME'):
        return

    for model_path in ('/proc/device-tree/model', '/sys/firmware/devicetree/base/model'):
        try:
            model = open(model_path, 'rb').read().decode('utf-8', 'ignore').strip('\x00')
        except OSError:
            continue

        # Jetson.GPIO 2.1.7 does not recognize the newer Orin Nano "Super"
        # model string yet, but the regular Orin Nano pin map is compatible.
        if 'Jetson Orin Nano' in model:
            os.environ['JETSON_MODEL_NAME'] = 'JETSON_ORIN_NANO'
            return


_ensure_supported_jetson_model()

try:
    from adafruit_extended_bus import ExtendedI2C as I2C

    from adafruit_bno08x import (
        BNO_REPORT_GAME_ROTATION_VECTOR,
        BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_LINEAR_ACCELERATION,
        BNO_REPORT_MAGNETOMETER,
        BNO_REPORT_ROTATION_VECTOR,
    )
    from adafruit_bno08x.i2c import BNO08X_I2C
except ImportError as exc:
    _IMPORT_ERROR = exc
    I2C = None
    BNO08X_I2C = None

    BNO_REPORT_GAME_ROTATION_VECTOR = None
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR = None
    BNO_REPORT_GYROSCOPE = None
    BNO_REPORT_LINEAR_ACCELERATION = None
    BNO_REPORT_MAGNETOMETER = None
    BNO_REPORT_ROTATION_VECTOR = None


def _normalize_quaternion(
    quat: Optional[Tuple[float, float, float, float]]
) -> Optional[Tuple[float, float, float, float]]:
    if quat is None:
        return None

    x, y, z, w = quat
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-9:
        return None

    return (x / norm, y / norm, z / norm, w / norm)


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class Bno085I2cNode(Node):
    _ORIENTATION_MODES = {
        'rotation': (BNO_REPORT_ROTATION_VECTOR, 'quaternion'),
        'game': (BNO_REPORT_GAME_ROTATION_VECTOR, 'game_quaternion'),
        'geomagnetic': (BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR, 'geomagnetic_quaternion'),
    }

    def __init__(self):
        super().__init__('bno085_i2c_node')

        if _IMPORT_ERROR is not None:
            raise RuntimeError(
                'Missing BNO085 Python dependencies: %s. '
                'Install with: python3 -m pip install --user '
                'adafruit-extended-bus adafruit-blinka adafruit-circuitpython-bno08x'
                % str(_IMPORT_ERROR)
            )

        self.declare_parameter('bus_num', 7)
        self.declare_parameter('address', 0x4A)
        self.declare_parameter('imu_topic', '/sensors/imu/raw')
        self.declare_parameter('mag_topic', '/sensors/mag')
        self.declare_parameter('frame_id', 'imu_frame')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('report_interval_us', 10000)
        self.declare_parameter('reconnect_period_sec', 1.0)
        self.declare_parameter('orientation_mode', 'geomagnetic')
        self.declare_parameter('publish_magnetic_field', False)
        self.declare_parameter('orientation_covariance', 0.02)
        self.declare_parameter('linear_acceleration_covariance', 0.5)
        self.declare_parameter('angular_velocity_covariance', 0.02)
        self.declare_parameter('magnetic_field_covariance', 0.05)

        self.bus_num = int(self.get_parameter('bus_num').value)
        self.address = int(self.get_parameter('address').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        mag_topic = str(self.get_parameter('mag_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.report_interval_us = max(1000, int(self.get_parameter('report_interval_us').value))
        self.reconnect_period_sec = max(0.1, float(self.get_parameter('reconnect_period_sec').value))
        self.publish_magnetic_field = _as_bool(self.get_parameter('publish_magnetic_field').value)

        orientation_mode = str(self.get_parameter('orientation_mode').value).strip().lower()
        if orientation_mode not in self._ORIENTATION_MODES:
            self.get_logger().warning(
                'Unknown orientation_mode "%s", falling back to "geomagnetic"' % orientation_mode
            )
            orientation_mode = 'geomagnetic'
        self.orientation_mode = orientation_mode
        self.orientation_report_id, self._orientation_attr = self._ORIENTATION_MODES[orientation_mode]

        if publish_rate_hz <= 0.0:
            publish_rate_hz = 100.0
        self.publish_period_sec = 1.0 / publish_rate_hz

        self.orientation_covariance = self._build_covariance(
            float(self.get_parameter('orientation_covariance').value)
        )
        self.linear_acceleration_covariance = self._build_covariance(
            float(self.get_parameter('linear_acceleration_covariance').value)
        )
        self.angular_velocity_covariance = self._build_covariance(
            float(self.get_parameter('angular_velocity_covariance').value)
        )
        self.magnetic_field_covariance = self._build_covariance(
            float(self.get_parameter('magnetic_field_covariance').value)
        )

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.imu_pub = self.create_publisher(Imu, imu_topic, sensor_qos)
        self.mag_pub = None
        if self.publish_magnetic_field:
            self.mag_pub = self.create_publisher(MagneticField, mag_topic, sensor_qos)

        self._i2c = None
        self._sensor = None
        self._last_connect_attempt = 0.0
        self._last_warn = {}

        self.create_timer(self.publish_period_sec, self._poll_sensor)

        self.get_logger().info(
            'BNO085 I2C node configured. bus=%d address=0x%02X imu_topic=%s '
            'orientation_mode=%s publish_magnetic_field=%s'
            % (
                self.bus_num,
                self.address,
                imu_topic,
                self.orientation_mode,
                str(self.publish_magnetic_field),
            )
        )

    def destroy_node(self):
        self._sensor = None
        self._i2c = None
        return super().destroy_node()

    def _build_covariance(self, value: float) -> List[float]:
        if value < 0.0:
            return [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return [value, 0.0, 0.0, 0.0, value, 0.0, 0.0, 0.0, value]

    def _warn_throttled(self, key: str, message: str, period_sec: float = 5.0):
        now = time.monotonic()
        if now - self._last_warn.get(key, 0.0) >= period_sec:
            self._last_warn[key] = now
            self.get_logger().warning(message)

    def _try_connect(self):
        now = time.monotonic()
        if now - self._last_connect_attempt < self.reconnect_period_sec:
            return
        self._last_connect_attempt = now

        try:
            i2c = I2C(self.bus_num)
            sensor = BNO08X_I2C(i2c, address=self.address)
            sensor.enable_feature(BNO_REPORT_GYROSCOPE, self.report_interval_us)
            sensor.enable_feature(BNO_REPORT_LINEAR_ACCELERATION, self.report_interval_us)
            sensor.enable_feature(self.orientation_report_id, self.report_interval_us)
            if self.publish_magnetic_field:
                sensor.enable_feature(BNO_REPORT_MAGNETOMETER, self.report_interval_us)

            self._i2c = i2c
            self._sensor = sensor
            self.get_logger().info(
                'Connected to BNO085 on /dev/i2c-%d at 0x%02X using %s fusion'
                % (self.bus_num, self.address, self.orientation_mode)
            )
        except Exception as exc:
            self._sensor = None
            self._i2c = None
            self._warn_throttled(
                'connect_error',
                'Unable to initialize BNO085 on /dev/i2c-%d at 0x%02X: %s'
                % (self.bus_num, self.address, str(exc)),
                period_sec=self.reconnect_period_sec,
            )

    def _poll_sensor(self):
        if self._sensor is None:
            self._try_connect()
            return

        try:
            quat = _normalize_quaternion(getattr(self._sensor, self._orientation_attr))
            if quat is None:
                self._warn_throttled(
                    'orientation_invalid',
                    'Received invalid %s quaternion from BNO085' % self.orientation_mode,
                )
                return

            gyro = self._sensor.gyro
            linear_accel = self._sensor.linear_acceleration

            stamp = self.get_clock().now().to_msg()

            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = self.frame_id
            imu_msg.orientation.x = quat[0]
            imu_msg.orientation.y = quat[1]
            imu_msg.orientation.z = quat[2]
            imu_msg.orientation.w = quat[3]
            imu_msg.orientation_covariance = list(self.orientation_covariance)
            imu_msg.angular_velocity.x = float(gyro[0])
            imu_msg.angular_velocity.y = float(gyro[1])
            imu_msg.angular_velocity.z = float(gyro[2])
            imu_msg.angular_velocity_covariance = list(self.angular_velocity_covariance)
            imu_msg.linear_acceleration.x = float(linear_accel[0])
            imu_msg.linear_acceleration.y = float(linear_accel[1])
            imu_msg.linear_acceleration.z = float(linear_accel[2])
            imu_msg.linear_acceleration_covariance = list(self.linear_acceleration_covariance)
            self.imu_pub.publish(imu_msg)

            if self.mag_pub is not None:
                mag = self._sensor.magnetic
                mag_msg = MagneticField()
                mag_msg.header.stamp = stamp
                mag_msg.header.frame_id = self.frame_id
                mag_msg.magnetic_field.x = float(mag[0]) * 1e-6
                mag_msg.magnetic_field.y = float(mag[1]) * 1e-6
                mag_msg.magnetic_field.z = float(mag[2]) * 1e-6
                mag_msg.magnetic_field_covariance = list(self.magnetic_field_covariance)
                self.mag_pub.publish(mag_msg)
        except Exception as exc:
            self._warn_throttled(
                'poll_error',
                'BNO085 read failed, reconnecting: %s' % str(exc),
            )
            self._sensor = None
            self._i2c = None


def main(args=None):
    rclpy.init(args=args)
    node = Bno085I2cNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
