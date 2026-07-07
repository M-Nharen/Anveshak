#!/usr/bin/env python3
"""
pwm_instant_cli.py
-------------------
Interactive ROS2 Jazzy terminal node for entering raw integer PWM values.

Usage (once running):
    Left,Right<ENTER>       e.g.  150,-100

Values are clamped to [-255, 255] and published on the 'cmd_pwm' topic
as a geometry_msgs/msg/Twist:
    msg.linear.x  -> Left PWM
    msg.angular.z -> Right PWM

Twist is reused here purely as a convenient pre-built message type to
avoid compiling a custom .msg interface -- the two fields we use have
no relation to actual linear/angular velocity.

Keyboard input is read on a dedicated background thread so it never
blocks rclpy's spin loop (and vice versa).
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

PWM_MIN = -255
PWM_MAX = 255


def clamp(value: int, lo: int = PWM_MIN, hi: int = PWM_MAX) -> int:
    return max(lo, min(hi, value))


class PwmInstantCli(Node):

    def __init__(self):
        super().__init__('pwm_instant_cli')

        self.publisher_ = self.create_publisher(Twist, 'cmd_pwm', 10)

        # Daemon thread so it dies automatically when the process exits,
        # without us needing to manually join it on Ctrl+C.
        self._input_thread = threading.Thread(
            target=self._input_loop, daemon=True
        )
        self._input_thread.start()

        self.get_logger().info(
            "PWM CLI ready. Enter 'Left,Right' (e.g. 150,-100) and press Enter."
        )

    def _input_loop(self):
        while rclpy.ok():
            try:
                raw = input('PWM> ').strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not raw:
                continue

            parsed = self._parse_input(raw)
            if parsed is None:
                print(
                    "  Invalid format. Expected: Left,Right "
                    "(e.g. 150,-100). Ints in [-255, 255]."
                )
                continue

            left, right = parsed
            self._publish_pwm(left, right)

    def _parse_input(self, raw: str):
        parts = raw.split(',')
        if len(parts) != 2:
            return None
        try:
            left_raw = int(parts[0].strip())
            right_raw = int(parts[1].strip())
        except ValueError:
            return None

        left = clamp(left_raw)
        right = clamp(right_raw)

        if left != left_raw or right != right_raw:
            print(
                f"  Note: clamped to safety bounds -> "
                f"Left: {left_raw} -> {left} | Right: {right_raw} -> {right}"
            )

        return left, right

    def _publish_pwm(self, left: int, right: int):
        msg = Twist()
        msg.linear.x = float(left)
        msg.angular.z = float(right)
        self.publisher_.publish(msg)
        print(f"  Sent -> Left: {left} | Right: {right}")


def main(args=None):
    rclpy.init(args=args)
    node = PwmInstantCli()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()