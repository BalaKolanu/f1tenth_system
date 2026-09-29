#!/usr/bin/env python3

"""Publish an autonomy-toggle request when Space is pressed in this terminal."""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class AutonomyKeyToggle(Node):
    def __init__(self):
        super().__init__('autonomy_key_toggle')
        self.declare_parameter('toggle_topic', '/safety/autonomy_toggle')
        toggle_topic = str(self.get_parameter('toggle_topic').value)
        self.publisher = self.create_publisher(Empty, toggle_topic, 10)
        self.get_logger().info(
            'Keyboard autonomy-toggle publisher ready on %s' % toggle_topic
        )

    def request_toggle(self):
        self.publisher.publish(Empty())
        self.get_logger().info('Space pressed; autonomy-toggle request published.')


def main(args=None):
    if not sys.stdin.isatty():
        raise RuntimeError('autonomy_key_toggle must run in an interactive terminal.')

    rclpy.init(args=args)
    node = AutonomyKeyToggle()
    terminal_settings = termios.tcgetattr(sys.stdin.fileno())

    print('Autonomy keyboard toggle: Space toggles autonomy; q exits.')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not readable:
                continue

            key = sys.stdin.read(1)
            if key == ' ':
                node.request_toggle()
            elif key.lower() == 'q':
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, terminal_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
