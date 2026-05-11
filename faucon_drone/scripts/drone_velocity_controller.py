#!/usr/bin/env python3
"""Convert Faucon drone velocity commands to simulated motor speeds."""

import math

from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

# ── Physical constants (must match drone_body.xacro + drone_plugins.xacro) ──
_MOTOR_K = 8.54858e-06   # N·s²/rad² (motorConstant)
_MASS = 1.512          # kg  (body 1.5 + 4×rotor 0.005 + camera/imu lumped)
_GRAVITY = 9.81           # m/s²
_MAX_RPM = 800.0          # rad/s (maxRotVelocity)
_MIN_RPM = 0.0

# Hover speed: F_hover = m·g → 4·k·ω² = m·g → ω = sqrt(m·g / 4k)
_HOVER = math.sqrt(_MASS * _GRAVITY / (4.0 * _MOTOR_K))  # ≈ 659 rad/s


class DroneVelocityController(Node):
    def __init__(self):
        super().__init__('drone_velocity_controller')

        # ── Tunable gains ────────────────────────────────────────────────
        self.declare_parameter('kz', 200.0)  # (rad/s) / (m/s)  vertical
        self.declare_parameter('kxy', 60.0)  # (rad/s) / (m/s)  horizontal
        self.declare_parameter('kyaw', 40.0)  # (rad/s) / (rad/s) yaw
        self.declare_parameter('cmd_timeout', 0.5)  # s — stop if no cmd

        self.kz = self.get_parameter('kz').value
        self.kxy = self.get_parameter('kxy').value
        self.kyaw = self.get_parameter('kyaw').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        self._armed = False
        self._last_cmd = self.get_clock().now()
        self._cmd = Twist()

        self._pub = self.create_publisher(Actuators, '/drone/command/motor_speed', 10)
        self._state_pub = self.create_publisher(String, '/faucon/drone/state', 10)
        self.create_subscription(Twist, '/faucon/drone/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Bool, '/faucon/drone/arm', self._enable_cb, 10)
        self.create_timer(1.0 / 50.0, self._loop)   # 50 Hz control loop
        self.create_timer(0.2, self._publish_state)

        self.get_logger().info(
            f'Drone velocity controller ready  '
            f'[hover={_HOVER:.0f} rad/s, kz={self.kz}, kxy={self.kxy}]'
        )

    def _enable_cb(self, msg: Bool):
        self._armed = msg.data
        if not self._armed:
            self._publish([0.0, 0.0, 0.0, 0.0])
            self.get_logger().info('Drone disarmed — motors stopped.')
        else:
            self.get_logger().info('Drone armed.')
        self._publish_state()

    def _cmd_cb(self, msg: Twist):
        self._cmd = msg
        self._last_cmd = self.get_clock().now()

    def _loop(self):
        if not self._armed:
            return

        age = (self.get_clock().now() - self._last_cmd).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            # Command stale — stop motors safely
            self._publish([0.0, 0.0, 0.0, 0.0])
            return

        vx = self._cmd.linear.x
        vy = self._cmd.linear.y
        vz = self._cmd.linear.z
        wz = self._cmd.angular.z

        dz = self.kz * vz
        dx = self.kxy * vx
        dy = self.kxy * vy
        dyaw = self.kyaw * wz

        # X-frame mixer — torque sign convention:
        #   CCW rotor (0,3) → +yaw torque  →  reduce to yaw right (+wz CW → −dyaw)
        #   CW  rotor (1,2) → −yaw torque  →  reduce to yaw right (+wz CW → +dyaw)
        s0 = _HOVER + dz + dx - dy - dyaw   # CCW  front-left
        s1 = _HOVER + dz + dx + dy + dyaw   # CW   front-right
        s2 = _HOVER + dz - dx - dy + dyaw   # CW   back-left
        s3 = _HOVER + dz - dx + dy - dyaw   # CCW  back-right

        self._publish([
            max(_MIN_RPM, min(_MAX_RPM, s))
            for s in [s0, s1, s2, s3]
        ])

    def _publish(self, speeds: list):
        msg = Actuators()
        msg.velocity = speeds
        self._pub.publish(msg)

    def _publish_state(self):
        msg = String()
        msg.data = 'ARMED' if self._armed else 'DISARMED'
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DroneVelocityController())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
