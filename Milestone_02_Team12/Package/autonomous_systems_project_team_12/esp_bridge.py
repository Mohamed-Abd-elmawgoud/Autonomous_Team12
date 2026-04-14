#!/usr/bin/env python3
"""
esp_bridge_node.py
ROS2 Jazzy node that:
  1. Subscribes to /steering  [Float64]  – steering normalised to -1…+1
  2. Subscribes to /velocity  [Float64]  – forward velocity in m/s
  3. Maps steering (-1..+1) → servo angle (60..120°, centre 90°)
  4. Maps velocity  (0..max) → ESC PWM (µs)
  5. Sends a compact binary packet to the ESP over UART
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial


class EspBridgeNode(Node):
    def __init__(self):
        super().__init__('esp_bridge_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate',   115200)
        self.declare_parameter('timeout',     0.1)

        # Steering servo angle range (degrees)
        self.declare_parameter('steer_centre_deg', 90)
        self.declare_parameter('steer_range_deg',  30)   # ±30° around centre

        # Throttle ESC PWM range (µs) — standard RC ESC protocol
        self.declare_parameter('throttle_min_us',  1000)
        self.declare_parameter('throttle_neutral_us', 1500)   
        self.declare_parameter('throttle_max_us',  2000)

        # Maximum expected velocity (m/s) — maps to throttle_max_us
        self.declare_parameter('max_velocity_ms', 0.5)

        port    = self.get_parameter('serial_port').value
        baud    = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value

        self.steer_centre = self.get_parameter('steer_centre_deg').value
        self.steer_range  = self.get_parameter('steer_range_deg').value
        self.thr_min      = self.get_parameter('throttle_min_us').value
        self.thr_neutral = self.get_parameter('throttle_neutral_us').value
        self.thr_max      = self.get_parameter('throttle_max_us').value
        self.max_vel      = self.get_parameter('max_velocity_ms').value

        # ── Latest values (updated independently by each subscriber) ──────
        self._steering = 0.0   # normalised -1…+1
        self._velocity = 0.0   # m/s

        # ── Serial port ───────────────────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=timeout)
            self.get_logger().info(f'Serial open: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial port {port}: {e}')
            raise

        # ── ROS subscribers ───────────────────────────────────────────────
        self.sub_steer = self.create_subscription(
            Float64,
            '/steering_angle',
            self._steering_cb,
            10
        )
        self.sub_vel = self.create_subscription(
            Float64,
            '/velocity',
            self._velocity_cb,
            10
        )

        # Safety: send neutral on startup
        self._send(0.0, 0.0)
        self.get_logger().info(
            f'EspBridgeNode ready  '
            f'steer: {self.steer_centre - self.steer_range}°–'
            f'{self.steer_centre + self.steer_range}°  '
            f'centre: {self.steer_centre}°  '
            f'max_vel: {self.max_vel} m/s'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────
    def _steering_cb(self, msg: Float64):
        self._steering = float(msg.data)
        self._send(self._steering, self._velocity)

    def _velocity_cb(self, msg: Float64):
        self._velocity = float(msg.data)
        self._send(self._steering, self._velocity)

    # ── Conversion + transmission ──────────────────────────────────────────
    def _send(self, steering: float, velocity: float):
        # Clamp steering to -1…+1
        steering = max(-1.0, min(1.0, steering))

        # Clamp velocity to -max_vel…+max_vel
        velocity = max(-self.max_vel, min(self.max_vel, velocity))

        # steering -1…+1  →  angle 60…120°
        steer_deg = int(round(self.steer_centre + steering * self.steer_range))
        steer_deg = max(self.steer_centre - self.steer_range,
                        min(self.steer_centre + self.steer_range, steer_deg))

        # velocity -max_vel…+max_vel  →  ESC PWM µs
        # 0 → thr_neutral (1500), +max → thr_max (2000), -max → thr_min (1000)
        throttle_norm = velocity / self.max_vel if self.max_vel > 0 else 0.0
        thr_us = int(self.thr_neutral + throttle_norm * (self.thr_max - self.thr_neutral))

        packet = self._build_packet(steer_deg, thr_us)
        try:
            self.ser.write(packet)
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

        self.get_logger().debug(
            f'TX  steer={steer_deg}°  thr_us={thr_us}  '
            f'(raw: steering={steering:.3f}  vel={velocity:.3f} m/s)'
        )

    @staticmethod
    def _build_packet(steer_deg: int, thr_us: int) -> bytes:
        """
        Binary packet format (8 bytes) — unchanged, matches ESP32 sketch:
          [0]    0xAA       – start byte 1
          [1]    0x55       – start byte 2
          [2]    uint8      – steering angle in degrees (60–120)
          [3]    0x00       – reserved / padding
          [4-5]  uint16 LE  – throttle PWM in µs (1000–2000)
          [6]    uint8      – XOR checksum of bytes [2-5]
          [7]    0x0A       – end marker
        """
        t_lo = thr_us & 0xFF
        t_hi = (thr_us >> 8) & 0xFF
        checksum = steer_deg ^ 0x00 ^ t_lo ^ t_hi
        return bytes([0xAA, 0x55, steer_deg, 0x00, t_lo, t_hi, checksum, 0x0A])

    def destroy_node(self):
        try:
            self._send(0.0, 0.0)   # centre steering, stop throttle on shutdown
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


# ── Entry point ────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = EspBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()