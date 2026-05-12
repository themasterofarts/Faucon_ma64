#!/usr/bin/env python3
"""
Arm, take off, then execute a configurable waypoint trajectory.

Phases
------
WAITING    → wait for /faucon/drone/odom and start_delay to expire
PREARM     → stream zero cmd_vel for prearm_setpoint_time, then arm + offboard
TAKEOFF    → climb to takeoff_altitude while holding spawn XY
TRAJECTORY → visit each (x, y, z) waypoint in world-ENU order
HOVER      → hold the final reached position indefinitely

Topics consumed
  /faucon/drone/odom   nav_msgs/Odometry   (position feedback)
  /faucon/drone/state  std_msgs/String     (DISARMED | ARMED | OFFBOARD | …)

Topics published
  /faucon/drone/cmd_vel              geometry_msgs/Twist   (body-ENU velocity)
  /faucon/drone/arm                  std_msgs/Bool
  /faucon/drone/trajectory/status    std_msgs/String       (phase label)

Parameters
----------
waypoints             list[float]  Flat [x0,y0,z0, x1,y1,z1, …] world-ENU (m).
                                   Empty list → take off and hover only.
waypoints_str         str          Comma-separated flat list "x0,y0,z0,x1,y1,z1,…".
                                   Used when waypoints is empty (convenient for
                                   launch-file / CLI: trajectory_waypoints:="…").
takeoff_altitude      float        3.0   Altitude (m) reached before first waypoint.
waypoint_tolerance    float        0.5   3-D arrival radius per waypoint (m).
cruise_speed          float        1.2   Max horizontal speed during a leg (m/s).
hover_kp_xy           float        0.6   Proportional gain for XY position hold.
hover_kp_z            float        0.7   Proportional gain for Z hold.
max_velocity_xy       float        2.0   Hard velocity cap, horizontal (m/s).
max_velocity_z        float        1.0   Hard velocity cap, vertical (m/s).
takeoff_speed         float        0.8   Climb rate during takeoff (m/s).
altitude_tolerance    float        0.15  Z tolerance to declare takeoff complete (m).
start_delay           float        20.0  Seconds to wait before arming (s).
prearm_setpoint_time  float        2.0   Seconds of zero setpoints before arm (s).
arm_retry_period      float        2.0   Interval between arm retries (s).
return_home           bool         False If true, append a final waypoint at the
                                         spawn XY / takeoff_altitude before hovering.
"""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class DroneTrajectory(Node):

    def __init__(self):
        super().__init__('drone_trajectory')

        self.declare_parameter('waypoints',            rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('waypoints_str',        '')
        self.declare_parameter('takeoff_altitude',     3.0)
        self.declare_parameter('waypoint_tolerance',   0.5)
        self.declare_parameter('cruise_speed',         1.2)
        self.declare_parameter('hover_kp_xy',          0.6)
        self.declare_parameter('hover_kp_z',           0.7)
        self.declare_parameter('max_velocity_xy',      2.0)
        self.declare_parameter('max_velocity_z',       1.0)
        self.declare_parameter('takeoff_speed',        0.8)
        self.declare_parameter('altitude_tolerance',   0.15)
        self.declare_parameter('start_delay',          20.0)
        self.declare_parameter('prearm_setpoint_time', 2.0)
        self.declare_parameter('arm_retry_period',     2.0)
        self.declare_parameter('return_home',          False)

        self._load_params()

        # Runtime state
        self._odom: Odometry | None = None
        self._drone_state = 'UNKNOWN'
        self._home_xy: tuple[float, float] | None = None
        self._phase = 'WAITING'
        self._wp_index = 0
        self._hover_target: tuple[float, float, float] | None = None
        self._start_time = self.get_clock().now()
        self._phase_started: rclpy.time.Time | None = None
        self._last_arm_request: rclpy.time.Time | None = None
        self._last_wait_log: rclpy.time.Time | None = None

        # Publishers / subscribers
        self._cmd_pub    = self.create_publisher(Twist,  '/faucon/drone/cmd_vel',           10)
        self._arm_pub    = self.create_publisher(Bool,   '/faucon/drone/arm',                10)
        self._status_pub = self.create_publisher(String, '/faucon/drone/trajectory/status',  10)
        self.create_subscription(Odometry, '/faucon/drone/odom',  self._odom_cb,   10)
        self.create_subscription(String,   '/faucon/drone/state', self._state_cb,  10)
        self.create_timer(1.0 / 20.0, self._loop)

        self.get_logger().info(
            f'DroneTrajectory ready — {len(self._waypoints)} waypoints, '
            f'takeoff_altitude={self._takeoff_altitude:.1f} m, '
            f'start_delay={self._start_delay:.1f} s, '
            f'return_home={self._return_home}'
        )
        if not self._waypoints:
            self.get_logger().warn(
                'No waypoints loaded (parameter "waypoints" is empty) — '
                'drone will take off and hover at takeoff_altitude.'
            )

    # ── parameter loading ──────────────────────────────────────────────────

    def _load_params(self):
        try:
            raw = list(self.get_parameter('waypoints').value or [])
        except Exception:
            raw = []

        if not raw:
            wps_str = str(self.get_parameter('waypoints_str').value).strip()
            if wps_str:
                try:
                    raw = [float(v) for v in wps_str.split(',') if v.strip()]
                except ValueError as exc:
                    raise ValueError(
                        f'"waypoints_str" must be comma-separated floats, got: {wps_str!r}'
                    ) from exc

        if len(raw) % 3 != 0:
            raise ValueError(
                f'"waypoints" must have a multiple of 3 elements [x,y,z per point], '
                f'got {len(raw)}: {raw}'
            )
        self._waypoints: list[tuple[float, float, float]] = [
            (float(raw[i]), float(raw[i + 1]), float(raw[i + 2]))
            for i in range(0, len(raw), 3)
        ]

        self._takeoff_altitude    = float(self.get_parameter('takeoff_altitude').value)
        self._waypoint_tolerance  = float(self.get_parameter('waypoint_tolerance').value)
        self._cruise_speed        = float(self.get_parameter('cruise_speed').value)
        self._hover_kp_xy         = float(self.get_parameter('hover_kp_xy').value)
        self._hover_kp_z          = float(self.get_parameter('hover_kp_z').value)
        self._max_velocity_xy     = float(self.get_parameter('max_velocity_xy').value)
        self._max_velocity_z      = float(self.get_parameter('max_velocity_z').value)
        self._takeoff_speed       = float(self.get_parameter('takeoff_speed').value)
        self._altitude_tolerance  = float(self.get_parameter('altitude_tolerance').value)
        self._start_delay         = float(self.get_parameter('start_delay').value)
        self._prearm_setpoint_time = float(self.get_parameter('prearm_setpoint_time').value)
        self._arm_retry_period    = float(self.get_parameter('arm_retry_period').value)
        self._return_home         = bool(self.get_parameter('return_home').value)

    # ── helpers ────────────────────────────────────────────────────────────

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, float(value)))

    @staticmethod
    def _yaw_from_quat(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

    def _is_armed(self) -> bool:
        return self._drone_state not in ('UNKNOWN', 'DISARMED')

    def _arm_request_due(self) -> bool:
        if self._last_arm_request is None:
            return True
        age = (self.get_clock().now() - self._last_arm_request).nanoseconds * 1e-9
        return age >= self._arm_retry_period

    def _current_pos(self) -> tuple[float, float, float]:
        p = self._odom.pose.pose.position
        return float(p.x), float(p.y), float(p.z)

    def _current_yaw(self) -> float:
        return self._yaw_from_quat(self._odom.pose.pose.orientation)

    def _enu_vel_to_body(
        self, vx_enu: float, vy_enu: float, yaw: float
    ) -> tuple[float, float]:
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        return cos_y * vx_enu + sin_y * vy_enu, -sin_y * vx_enu + cos_y * vy_enu

    def _vel_toward(
        self, tx: float, ty: float, tz: float
    ) -> tuple[float, float, float]:
        """Body-ENU cmd_vel components toward target (tx,ty,tz)."""
        cx, cy, cz = self._current_pos()
        yaw = self._current_yaw()

        ex, ey, ez = tx - cx, ty - cy, tz - cz
        dist_xy = math.hypot(ex, ey)

        speed = min(self._cruise_speed, self._hover_kp_xy * dist_xy)
        if dist_xy > 1e-4:
            vx_enu = speed * ex / dist_xy
            vy_enu = speed * ey / dist_xy
        else:
            vx_enu = vy_enu = 0.0

        vz = self._clamp(self._hover_kp_z * ez, self._max_velocity_z)
        vx_body, vy_body = self._enu_vel_to_body(vx_enu, vy_enu, yaw)

        return (
            self._clamp(vx_body, self._max_velocity_xy),
            self._clamp(vy_body, self._max_velocity_xy),
            vz,
        )

    def _distance_to(self, tx: float, ty: float, tz: float) -> float:
        cx, cy, cz = self._current_pos()
        return math.sqrt((tx - cx) ** 2 + (ty - cy) ** 2 + (tz - cz) ** 2)

    # ── publishers ─────────────────────────────────────────────────────────

    def _publish_cmd(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.linear.y  = float(vy)
        msg.linear.z  = float(vz)
        msg.angular.z = float(yaw_rate)
        self._cmd_pub.publish(msg)

    def _publish_status(self, label: str):
        msg = String()
        msg.data = label
        self._status_pub.publish(msg)

    def _send_arm(self, arm: bool = True):
        msg = Bool()
        msg.data = arm
        self._arm_pub.publish(msg)
        self._last_arm_request = self.get_clock().now()
        verb = 'Arm' if arm else 'Disarm'
        self.get_logger().info(f'{verb} request sent.')

    # ── callbacks ──────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._odom = msg
        if self._home_xy is None:
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            self._home_xy = (x, y)
            self.get_logger().info(f'Home position set at x={x:.2f} m, y={y:.2f} m')

    def _state_cb(self, msg: String):
        self._drone_state = msg.data

    # ── main loop (20 Hz) ──────────────────────────────────────────────────

    def _loop(self):
        if self._odom is None:
            self._publish_cmd()
            self._publish_status('WAITING')
            now = self.get_clock().now()
            if (self._last_wait_log is None or
                    (now - self._last_wait_log).nanoseconds * 1e-9 >= 2.0):
                self.get_logger().info('Waiting for /faucon/drone/odom…')
                self._last_wait_log = now
            return

        if self._elapsed() < self._start_delay:
            self._publish_cmd()
            self._publish_status('WAITING')
            return

        if self._phase == 'WAITING':
            self._phase = 'PREARM'
            self._phase_started = self.get_clock().now()
            self.get_logger().info(
                f'start_delay elapsed. Pre-streaming setpoints for '
                f'{self._prearm_setpoint_time:.1f} s before arming…'
            )

        self._step()

    def _step(self):
        if self._phase == 'PREARM':
            self._publish_cmd()
            self._publish_status('PREARM')
            age = (self.get_clock().now() - self._phase_started).nanoseconds * 1e-9
            if age >= self._prearm_setpoint_time:
                if self._arm_request_due():
                    self._send_arm()
                if self._is_armed():
                    self._phase = 'TAKEOFF'
                    self.get_logger().info(
                        f'Armed. Taking off to {self._takeoff_altitude:.1f} m…'
                    )

        elif self._phase == 'TAKEOFF':
            self._publish_status('TAKEOFF')
            if not self._is_armed() and self._arm_request_due():
                self._send_arm()

            _, _, cz = self._current_pos()
            hx, hy = self._home_xy

            if cz >= self._takeoff_altitude - self._altitude_tolerance:
                self._on_takeoff_complete()
            else:
                remaining = max(0.0, self._takeoff_altitude - cz)
                climb = self._clamp(
                    min(self._takeoff_speed, max(0.25, 0.6 * remaining)),
                    self._max_velocity_z,
                )
                vx, vy, _ = self._vel_toward(hx, hy, self._takeoff_altitude)
                self._publish_cmd(vx, vy, climb)

        elif self._phase == 'TRAJECTORY':
            if self._wp_index >= len(self._waypoints):
                cx, cy, cz = self._current_pos()
                self._hover_target = (cx, cy, cz)
                self._phase = 'HOVER'
                self.get_logger().info(
                    f'All {len(self._waypoints)} waypoints reached. '
                    f'Holding at ({cx:.2f}, {cy:.2f}, {cz:.2f}) m.'
                )
                return

            tx, ty, tz = self._waypoints[self._wp_index]
            n = len(self._waypoints)
            self._publish_status(f'WP_{self._wp_index + 1}/{n}')

            if self._distance_to(tx, ty, tz) < self._waypoint_tolerance:
                self.get_logger().info(
                    f'Waypoint {self._wp_index + 1}/{n} reached '
                    f'({tx:.1f}, {ty:.1f}, {tz:.1f}) m.'
                )
                self._wp_index += 1
            else:
                vx, vy, vz = self._vel_toward(tx, ty, tz)
                self._publish_cmd(vx, vy, vz)

        elif self._phase == 'HOVER':
            self._publish_status('HOVER')
            tx, ty, tz = self._hover_target
            vx, vy, vz = self._vel_toward(tx, ty, tz)
            self._publish_cmd(vx, vy, vz)

    def _on_takeoff_complete(self):
        _, _, cz = self._current_pos()
        self.get_logger().info(f'Takeoff complete at z={cz:.2f} m.')

        effective_wps = list(self._waypoints)
        if self._return_home:
            hx, hy = self._home_xy
            effective_wps.append((hx, hy, self._takeoff_altitude))
            self.get_logger().info(
                f'return_home=true: appended home waypoint '
                f'({hx:.2f}, {hy:.2f}, {self._takeoff_altitude:.2f}) m.'
            )
        self._waypoints = effective_wps

        if effective_wps:
            self._phase = 'TRAJECTORY'
            self._wp_index = 0
            self.get_logger().info(
                f'Starting trajectory: {len(effective_wps)} waypoints.'
            )
        else:
            cx, cy, cz = self._current_pos()
            self._hover_target = (cx, cy, cz)
            self._phase = 'HOVER'
            self.get_logger().info(
                f'No waypoints — hovering at ({cx:.2f}, {cy:.2f}, {cz:.2f}) m.'
            )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DroneTrajectory())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
