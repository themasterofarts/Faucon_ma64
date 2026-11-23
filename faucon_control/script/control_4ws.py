#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool, Float64MultiArray
import math

from rclpy.duration import Duration


class CmdVelToJoints(Node):
    def __init__(self):
        super().__init__("cmd_vel_based_4ws_control")

        get_logger = self.get_logger()
        get_logger.info("4WS Control Node Initialized")

        # Parameters (matched to your URDF)
        self.declare_parameter("wheel_radius", 0.25)  # Wheel radius in meters
        self.declare_parameter(
            "wheel_base", 0.65
        )  # Distance between front and rear axles
        self.declare_parameter("track_width", 1.0)  # Left-right wheel distance
        self.declare_parameter("max_steer", 0.7854)  # Steering limit (rad, 45 deg)
        self.declare_parameter("steer_gain", 1.0)  # Gain for steering angle
        self.declare_parameter("wheel_gain", 1.0)  # Gain for wheel velocity
        self.declare_parameter("max_steer_rate", 0.5)  # Steering joint limit (rad/s)
        self.declare_parameter("steer_time_from_start", 0.2)
        # Securite : arret des roues si aucune commande /cmd_vel recue depuis cmd_timeout (s)
        self.declare_parameter("cmd_timeout", 0.5)

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_base = self.get_parameter("wheel_base").value
        self.track_width = self.get_parameter("track_width").value
        self.max_steer = self.get_parameter("max_steer").value
        self.steer_gain = self.get_parameter("steer_gain").value
        self.wheel_gain = self.get_parameter("wheel_gain").value
        self.max_steer_rate = self.get_parameter("max_steer_rate").value
        self.steer_time_from_start = self.get_parameter("steer_time_from_start").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value

        self.last_steer_positions = [0.0, 0.0, 0.0, 0.0]
        self.last_cmd_time = self.get_clock().now()
        # Instant de reception de la derniere commande (pour le watchdog)
        self.last_cmd_rx_time = None
        self.wheels_stopped = True
        # Arret d'urgence (/e_stop) : roues a zero et /cmd_vel ignore tant qu'il est actif
        self.e_stop_active = False

        # Publisher for steering (JointTrajectory for JointTrajectoryController)
        self.steering_pub = self.create_publisher(
            JointTrajectory, "/steer_controller/joint_trajectory", 10
        )
        # Publisher for wheels (Float64MultiArray for JointGroupVelocityController)
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, "/velocity_controller/commands", 10
        )

        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)
        # twist_mux bloque les entrees mais ne publie pas de zero : on arrete
        # donc les roues ici, immediatement, sans attendre le watchdog.
        self.create_subscription(Bool, "/e_stop", self.e_stop_cb, 10)

        # Watchdog : le JointGroupVelocityController conserve la derniere consigne,
        # il faut donc publier explicitement un zero si les commandes s'arretent.
        self.create_timer(0.1, self._watchdog_cb)

        # Joint ordering matches steer_controllers.yaml
        self.steer_joints = [
            "left_wheel_leg_joint",
            "right_wheel_leg_joint",
            "left_wheel_back_leg_joint",
            "right_wheel_back_leg_joint",
        ]
        self.wheel_joints = [
            "left_wheel_joint",
            "right_wheel_joint",
            "wheel_back_left_joint",
            "wheel_back_right_joint",
        ]

    def _publish_zero_wheels(self):
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [0.0] * len(self.wheel_joints)
        self.wheel_pub.publish(wheel_msg)
        self.wheels_stopped = True

    def e_stop_cb(self, msg: Bool):
        active = bool(msg.data)
        if active:
            # Republie un zero a chaque message (l'IHM republie l'etat a 1 Hz)
            self._publish_zero_wheels()
            if not self.e_stop_active:
                self.get_logger().error("ARRET D'URGENCE actif : roues a zero, /cmd_vel ignore.")
        elif self.e_stop_active:
            self.get_logger().warn("Arret d'urgence rearme : /cmd_vel de nouveau accepte.")
        self.e_stop_active = active

    def _watchdog_cb(self):
        if self.wheels_stopped or self.last_cmd_rx_time is None or self.cmd_timeout <= 0.0:
            return
        age = (self.get_clock().now() - self.last_cmd_rx_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self._publish_zero_wheels()
            self.get_logger().warn(
                f"Aucune commande /cmd_vel depuis {age:.2f} s : arret des roues."
            )

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd_rx_time = self.get_clock().now()
        if self.e_stop_active:
            if not self.wheels_stopped:
                self._publish_zero_wheels()
            return
        vx = msg.linear.x
        wz = msg.angular.z

        EPS_VX = 1e-3
        EPS_WZ = 1e-3

        wheel_positions = [
            (self.wheel_base / 2.0, self.track_width / 2.0),    # front left
            (self.wheel_base / 2.0, -self.track_width / 2.0),   # front right
            (-self.wheel_base / 2.0, self.track_width / 2.0),   # rear left
            (-self.wheel_base / 2.0, -self.track_width / 2.0),  # rear right
        ]

        if abs(vx) < EPS_VX and abs(wz) < EPS_WZ:
            target_steers = [0.0, 0.0, 0.0, 0.0]
        else:
            target_steers = []

            for (wheel_x, wheel_y), previous in zip(wheel_positions, self.last_steer_positions):
                wheel_vx = vx - wz * wheel_y
                wheel_vy = wz * wheel_x
                steer = self._wheel_steer_angle(wheel_vx, wheel_vy, previous) * self.steer_gain
                steer = self._clamp(steer, -self.max_steer, self.max_steer)
                target_steers.append(steer)

        steer_positions = self._limit_steer_rate(target_steers)
        wheel_speeds = self._compute_wheel_speeds(vx, wz, steer_positions, wheel_positions)

        jt_steer = JointTrajectory()
        jt_steer.joint_names = self.steer_joints
        pt = JointTrajectoryPoint()
        pt.positions = steer_positions
        pt.time_from_start = Duration(seconds=self.steer_time_from_start).to_msg()
        jt_steer.points = [pt]
        self.steering_pub.publish(jt_steer)

        wheel_msg = Float64MultiArray()
        wheel_msg.data = wheel_speeds
        self.wheel_pub.publish(wheel_msg)
        self.wheels_stopped = all(abs(w) < 1e-9 for w in wheel_speeds)

    def _compute_wheel_speeds(self, vx, wz, steer_positions, wheel_positions):
        wheel_speeds = []
        for steer, (wheel_x, wheel_y) in zip(steer_positions, wheel_positions):
            wheel_vx = vx - wz * wheel_y
            wheel_vy = wz * wheel_x

            # Project the desired wheel velocity onto the achievable steering axis.
            linear_speed = wheel_vx * math.cos(steer) + wheel_vy * math.sin(steer)
            angular_speed = (
                linear_speed / self.wheel_radius
                if abs(self.wheel_radius) > 1e-6
                else 0.0
            )
            wheel_speeds.append(angular_speed * self.wheel_gain)

        return wheel_speeds

    def _limit_steer_rate(self, target_positions):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9
        self.last_cmd_time = now

        if dt <= 0.0:
            return self.last_steer_positions

        max_delta = abs(self.max_steer_rate) * dt
        limited_positions = []
        for current, target in zip(self.last_steer_positions, target_positions):
            delta = self._clamp(target - current, -max_delta, max_delta)
            limited_positions.append(current + delta)

        self.last_steer_positions = limited_positions
        return limited_positions

    @staticmethod
    def _wheel_steer_angle(wheel_vx, wheel_vy, previous=0.0):
        """Angle de braquage dans [-pi/2, pi/2] pour la vitesse roue (vx, vy).

        atan2 renvoie un angle dans [-pi, pi] : en marche arriere (vx < 0) il vaut
        ~ +/-pi, ce qui, une fois borne a max_steer, braquait les roues a 45 deg
        (deplacement en crabe). Une roue pouvant tourner dans les deux sens,
        braquer a (angle - pi) et inverser la vitesse est equivalent : on replie
        donc l'angle dans [-pi/2, pi/2]. Le signe de la vitesse roue est ensuite
        obtenu par projection dans _compute_wheel_speeds.
        Pres de +/-pi/2 (deux solutions equivalentes), on garde la plus proche
        du braquage precedent pour eviter que les roues oscillent d'une butee a l'autre.
        """
        raw = math.atan2(wheel_vy, wheel_vx)
        candidates = [a for a in (raw, raw - math.pi, raw + math.pi)
                      if -math.pi / 2 - 1e-9 <= a <= math.pi / 2 + 1e-9]
        return min(candidates, key=lambda a: abs(a - previous))

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))



def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToJoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
