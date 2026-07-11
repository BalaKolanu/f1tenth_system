#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class AutonomyToggle(Node):
    def __init__(self):
        super().__init__('autonomy_toggle')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('button_index', 5)
        self.declare_parameter('enabled_topic', '/safety/autonomy_enabled')
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('startup_enabled', False)

        self.joy_topic = str(self.get_parameter('joy_topic').value)
        self.button_index = int(self.get_parameter('button_index').value)
        self.enabled_topic = str(self.get_parameter('enabled_topic').value)
        publish_hz = max(1.0, float(self.get_parameter('publish_hz').value))

        self.enabled = _as_bool(self.get_parameter('startup_enabled').value)
        self.last_button_pressed = False

        self.pub = self.create_publisher(Bool, self.enabled_topic, 10)
        self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)
        self.create_timer(1.0 / publish_hz, self.publish_state)

        self.get_logger().info(
            'Autonomy toggle ready. button=%d joy=%s topic=%s startup=%s'
            % (
                self.button_index,
                self.joy_topic,
                self.enabled_topic,
                str(self.enabled).lower(),
            )
        )

    def joy_callback(self, msg: Joy):
        pressed = (
            0 <= self.button_index < len(msg.buttons)
            and msg.buttons[self.button_index] == 1
        )

        if pressed and not self.last_button_pressed:
            self.enabled = not self.enabled
            state = 'ENABLED' if self.enabled else 'DISABLED'
            self.get_logger().info('Autonomy toggle switched: %s' % state)

        self.last_button_pressed = pressed

    def publish_state(self):
        msg = Bool()
        msg.data = self.enabled
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyToggle()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
