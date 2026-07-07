#!/usr/bin/env python3
"""
serial_bridge_node.py
----------------------
ROS2 Jazzy hardware gateway bridge.

Downstream (Jetson -> ESP32):
    Subscribes to 'cmd_pwm' (geometry_msgs/msg/Twist), packs
    linear.x / angular.z into a JetsonToEspPacket protobuf, COBS-frames
    it, and writes it straight to /dev/ttyUSB0.

Upstream (ESP32 -> Jetson):
    A dedicated background thread continuously reads raw bytes from the
    same serial port. On each 0x00 delimiter it COBS-decodes the frame,
    parses it as an EspToJetsonPacket, and prints the embedded log text.

Both directions share the *same* physical UART. This works safely
because:
  1. All traffic (both directions) is COBS-framed, so 0x00 is
     guaranteed to only ever appear as a frame delimiter -- never
     inside payload bytes. This is what lets binary protobuf and
     "logging" coexist on one wire without corruption.
  2. Writes are protected by a lock so a downstream PWM write can never
     interleave with itself from another thread.
  3. Reads only ever happen on the single RX thread, so there is no
     read/read race on the port.
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial
from cobs import cobs

import pwm_proto_pb2

SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 115200
SERIAL_TIMEOUT = 0.001  # seconds; near-zero so the RX thread never blocks

DELIMITER = b'\x00'


class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('serial_bridge_node')

        try:
            self._ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                timeout=SERIAL_TIMEOUT,
                write_timeout=SERIAL_TIMEOUT,
            )
        except serial.SerialException as exc:
            self.get_logger().error(
                f"Failed to open {SERIAL_PORT} @ {BAUD_RATE}: {exc}"
            )
            raise

        self._write_lock = threading.Lock()
        self._rx_buffer = bytearray()
        self._stop_event = threading.Event()

        self._subscription = self.create_subscription(
            Twist, 'cmd_pwm', self._on_cmd_pwm, 10
        )

        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True
        )
        self._rx_thread.start()

        self.get_logger().info(
            f"Serial bridge online on {SERIAL_PORT} @ {BAUD_RATE} baud."
        )

    # ------------------------------------------------------------------
    # Downstream: ROS -> ESP32
    # ------------------------------------------------------------------
    def _on_cmd_pwm(self, msg: Twist):
        left = int(round(msg.linear.x))
        right = int(round(msg.angular.z))

        packet = pwm_proto_pb2.JetsonToEspPacket()
        packet.pwm_command.left_pwm = left
        packet.pwm_command.right_pwm = right

        raw = packet.SerializeToString()
        framed = cobs.encode(raw) + DELIMITER

        with self._write_lock:
            try:
                self._ser.write(framed)
                self._ser.flush()
            except serial.SerialException as exc:
                self.get_logger().warn(f"Serial write failed: {exc}")

    # ------------------------------------------------------------------
    # Upstream: ESP32 -> ROS / console
    # ------------------------------------------------------------------
    def _rx_loop(self):
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                n_waiting = self._ser.in_waiting
                chunk = self._ser.read(n_waiting if n_waiting else 1)
            except serial.SerialException as exc:
                self.get_logger().warn(f"Serial read failed: {exc}")
                continue

            if not chunk:
                continue

            for byte in chunk:
                if byte == 0x00:
                    self._handle_frame(bytes(self._rx_buffer))
                    self._rx_buffer.clear()
                else:
                    self._rx_buffer.append(byte)
                    # Guard against a corrupted stream that never sees a
                    # delimiter -- drop the buffer rather than growing
                    # unbounded.
                    if len(self._rx_buffer) > 4096:
                        self._rx_buffer.clear()

    def _handle_frame(self, framed_bytes: bytes):
        if not framed_bytes:
            return

        try:
            raw = cobs.decode(framed_bytes)
        except cobs.DecodeError:
            return

        packet = pwm_proto_pb2.EspToJetsonPacket()
        try:
            packet.ParseFromString(raw)
        except Exception:
            return

        if packet.WhichOneof('packet_type') == 'log_message':
            self._print_log(packet.log_message.text)

    def _print_log(self, text: str):
        # Non-blocking, direct write: clear the current line then print
        # the ESP32 log so it never gets tangled with any in-progress
        # terminal input line.
        sys.stdout.write('\r\x1b[K')
        sys.stdout.write(f"[ESP32] {text}\n")
        sys.stdout.flush()

    def destroy_node(self):
        self._stop_event.set()
        if self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        if self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()