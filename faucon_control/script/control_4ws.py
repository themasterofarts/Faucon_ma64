#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
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

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_base = self.get_parameter("wheel_base").value
        self.track_width = self.get_parameter("track_width").value
        self.max_steer = self.get_parameter("max_steer").value
        self.steer_gain = self.get_parameter("steer_gain").value
        self.wheel_gain = self.get_parameter("wheel_gain").value
        self.max_steer_rate = self.get_parameter("max_steer_rate").value
        self.steer_time_from_start = self.get_parameter("steer_time_from_start").value

        self.last_steer_positions = [0.0, 0.0, 0.0, 0.0]
        self.last_cmd_time = self.get_clock().now()

        # Publisher for steering (JointTrajectory for JointTrajectoryController)
        self.steering_pub = self.create_publisher(
            JointTrajectory, "/steer_controller/joint_trajectory", 10
        )
        # Publisher for wheels (Float64MultiArray for JointGroupVelocityController)
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, "/velocity_controller/commands", 10
        )

        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)

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

    def cmd_vel_cb(self, msg: Twist):
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

            for wheel_x, wheel_y in wheel_positions:
                wheel_vx = vx - wz * wheel_y
                wheel_vy = wz * wheel_x
                steer = math.atan2(wheel_vy, wheel_vx) * self.steer_gain
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
