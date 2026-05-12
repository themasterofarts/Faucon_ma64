#!/usr/bin/env python3
"""Arm a Faucon drone, take off, then hold a stationary hover."""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class DroneTakeoffHover(Node):
    def __init__(self):
        super().__init__('drone_takeoff_hover')

        self.declare_parameter('target_altitude', 3.0)
        self.declare_parameter('start_delay', 20.0)
        self.declare_parameter('prearm_setpoint_time', 2.0)
        self.declare_parameter('takeoff_speed', 0.8)
        self.declare_parameter('hover_kp_xy', 0.6)
        self.declare_parameter('hover_kp_z', 0.7)
        self.declare_parameter('max_velocity_xy', 0.8)
        self.declare_parameter('max_velocity_z', 0.8)
        self.declare_parameter('altitude_tolerance', 0.15)
        self.declare_parameter('arm_retry_period', 2.0)

        self.target_altitude = float(self.get_parameter('target_altitude').value)
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.prearm_setpoint_time = float(
            self.get_parameter('prearm_setpoint_time').value)
        self.takeoff_speed = float(self.get_parameter('takeoff_speed').value)
        self.hover_kp_xy = float(self.get_parameter('hover_kp_xy').value)
        self.hover_kp_z = float(self.get_parameter('hover_kp_z').value)
        self.max_velocity_xy = float(self.get_parameter('max_velocity_xy').value)
        self.max_velocity_z = float(self.get_parameter('max_velocity_z').value)
        self.altitude_tolerance = float(
            self.get_parameter('altitude_tolerance').value)
        self.arm_retry_period = float(self.get_parameter('arm_retry_period').value)

        self._odom = None
        self._state = 'UNKNOWN'
        self._home_x = None
        self._home_y = None
        self._phase = 'WAITING'
        self._start_time = self.get_clock().now()
        self._last_arm_request = None
        self._last_wait_log = None

        self._cmd_pub = self.create_publisher(Twist, '/faucon/drone/cmd_vel', 10)
        self._arm_pub = self.create_publisher(Bool, '/faucon/drone/arm', 10)
        self.create_subscription(Odometry, '/faucon/drone/odom', self._odom_cb, 10)
        self.create_subscription(String, '/faucon/drone/state', self._state_cb, 10)
        self.create_timer(1.0 / 20.0, self._loop)

        self.get_logger().info(
            'Auto takeoff hover ready '
            f'[target_altitude={self.target_altitude:.1f} m, '
            f'start_delay={self.start_delay:.1f} s]'
        )

    def _odom_cb(self, msg: Odometry):
        self._odom = msg
        if self._home_x is None:
            self._home_x = float(msg.pose.pose.position.x)
            self._home_y = float(msg.pose.pose.position.y)
            self.get_logger().info(
                f'Hover reference set at x={self._home_x:.2f}, y={self._home_y:.2f}')

    def _state_cb(self, msg: String):
        self._state = msg.data

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, float(value)))

    @staticmethod
    def _yaw_from_quat(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _elapsed(self):
        return (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

    def _send_arm(self):
        msg = Bool()
        msg.data = True
        self._arm_pub.publish(msg)
        self._last_arm_request = self.get_clock().now()
        self.get_logger().info('Arm/offboard request sent.')

    def _is_armed_or_offboard(self):
        return self._state not in ('UNKNOWN', 'DISARMED')

    def _arm_request_due(self):
        if self._last_arm_request is None:
            return True
        age = (self.get_clock().now() - self._last_arm_request).nanoseconds * 1e-9
        return age >= self.arm_retry_period

    def _log_waiting_for_odom(self):
        now = self.get_clock().now()
        if self._last_wait_log is None:
            due = True
        else:
            due = (now - self._last_wait_log).nanoseconds * 1e-9 >= 2.0
        if due:
            self.get_logger().info('Waiting for /faucon/drone/odom...')
            self._last_wait_log = now

    def _publish_cmd(self, vx_body=0.0, vy_body=0.0, vz=0.0, yaw_rate=0.0):
        msg = Twist()
        msg.linear.x = float(vx_body)
        msg.linear.y = float(vy_body)
        msg.linear.z = float(vz)
        msg.angular.z = float(yaw_rate)
        self._cmd_pub.publish(msg)

    def _hold_command(self, climb_speed=None):
        if self._odom is None or self._home_x is None:
            self._publish_cmd()
            return

        pos = self._odom.pose.pose.position
        q = self._odom.pose.pose.orientation
        yaw = self._yaw_from_quat(q)

        vx_enu = self._clamp(
            self.hover_kp_xy * (self._home_x - float(pos.x)),
            self.max_velocity_xy)
        vy_enu = self._clamp(
            self.hover_kp_xy * (self._home_y - float(pos.y)),
            self.max_velocity_xy)

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        vx_body = cos_y * vx_enu + sin_y * vy_enu
        vy_body = -sin_y * vx_enu + cos_y * vy_enu

        if climb_speed is None:
            vz = self.hover_kp_z * (self.target_altitude - float(pos.z))
        else:
            vz = climb_speed
        vz = self._clamp(vz, self.max_velocity_z)
        self._publish_cmd(vx_body, vy_body, vz)

    def _loop(self):
        elapsed = self._elapsed()

        if self._odom is None:
            if elapsed > 5.0 and self._phase == 'WAITING':
                self._log_waiting_for_odom()
            self._publish_cmd()
            return

        if elapsed < self.start_delay:
            self._publish_cmd()
            return

        if self._phase == 'WAITING':
            self._phase = 'PREARM'
            self._phase_started = self.get_clock().now()
            self.get_logger().info('Pre-streaming offboard setpoints before arming.')

        if self._phase == 'PREARM':
            self._publish_cmd()
            phase_age = (
                self.get_clock().now() - self._phase_started).nanoseconds * 1e-9
            if phase_age >= self.prearm_setpoint_time:
                if self._arm_request_due():
                    self._send_arm()
                if self._is_armed_or_offboard():
                    self._phase = 'TAKEOFF'
                    self.get_logger().info('Takeoff started.')
            return

        if self._phase == 'TAKEOFF':
            if not self._is_armed_or_offboard() and self._arm_request_due():
                self._send_arm()
            alt = float(self._odom.pose.pose.position.z)
            if alt >= self.target_altitude - self.altitude_tolerance:
                self._phase = 'HOVER'
                self.get_logger().info(
                    f'Hover reached at z={alt:.2f} m. Holding station.')
                self._hold_command()
            else:
                remaining = max(0.0, self.target_altitude - alt)
                climb = min(self.takeoff_speed, max(0.25, 0.6 * remaining))
                self._hold_command(climb_speed=climb)
            return

        self._hold_command()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DroneTakeoffHover())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
