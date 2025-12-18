#!/usr/bin/env python3

"""
4WS CmdVel Controller with PID (Steering + Wheels)
------------------
This node converts /cmd_vel into joint commands using CLOSED-LOOP PID control.
- Steering joints are controlled in POSITION (JointTrajectory)
- Wheel joints are controlled in VELOCITY (Float64MultiArray)

Faucon project
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# PID CLASS
class PID:
    def __init__(self, kp, ki, kd, limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.limit = limit

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0.0 else 0.0
        self.prev_error = error

        u = self.kp * error + self.ki * self.integral + self.kd * derivative

        if self.limit is not None:
            u = max(-self.limit, min(self.limit, u))
        return u


# MAIN NODE
class CmdVelToJointsPID(Node):
    def __init__(self):
        super().__init__("cmd_vel_4ws_pid_control")
        self.get_logger().info("4WS PID Control Node Initialized")

        # PARAMETERS
        self.declare_parameter("wheel_radius", 0.35)
        self.declare_parameter("wheel_base", 1.4)
        self.declare_parameter("track_width", 1.2)
        self.declare_parameter("max_steer", math.pi / 3)
        self.declare_parameter("dt", 0.01)

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_base = self.get_parameter("wheel_base").value
        self.track_width = self.get_parameter("track_width").value
        self.max_steer = self.get_parameter("max_steer").value
        self.dt = self.get_parameter("dt").value

        # JOINT NAMES
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

        # STATE STORAGE
        self.steer_pos = {j: 0.0 for j in self.steer_joints}
        self.wheel_vel = {j: 0.0 for j in self.wheel_joints}

        # PID CONTROLLERS
        self.pid_steer = {
            j: PID(3.0, 0.1, 0.05, limit=1.0) for j in self.steer_joints
        }

        self.pid_wheel = {
            j: PID(2.0, 0.0, 0.05, limit=20.0) for j in self.wheel_joints
        }

        # ROS INTERFACES
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)

        self.steer_pub = self.create_publisher(
            JointTrajectory, "/steer_controller/joint_trajectory", 10
        )

        self.wheel_pub = self.create_publisher(
            Float64MultiArray, "/velocity_controller/commands", 10
        )

    # JOINT FEEDBACK
    def joint_state_cb(self, msg: JointState):
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name in self.steer_pos:
                self.steer_pos[name] = pos
            if name in self.wheel_vel:
                self.wheel_vel[name] = vel

    # CMD_VEL CALLBACK
    def cmd_vel_cb(self, msg: Twist):
        vx = msg.linear.x
        wz = msg.angular.z

        # STEERING COMPUTATION
        if abs(wz) < 1e-4:
            steer = 0.0
        elif abs(vx) > 1e-4:
            steer = math.atan2(self.wheel_base * wz, vx)
        else:
            steer = math.copysign(self.max_steer * 0.8, wz)

        steer = max(-self.max_steer, min(self.max_steer, steer))
        targets_steer = {
            "left_wheel_leg_joint": steer,
            "right_wheel_leg_joint": steer,
            "left_wheel_back_leg_joint": -steer,
            "right_wheel_back_leg_joint": -steer,
        }

        # STEERING PID
        jt = JointTrajectory()
        jt.joint_names = self.steer_joints
        pt = JointTrajectoryPoint()

        for j in self.steer_joints:
            error = targets_steer[j] - self.steer_pos[j]
            u = self.pid_steer[j].compute(error, self.dt)
            pt.positions.append(self.steer_pos[j] + u)

        pt.time_from_start = Duration(seconds=0.1).to_msg()
        jt.points = [pt]
        self.steer_pub.publish(jt)

        # WHEEL SPEED TARGETS
        omega = vx / self.wheel_radius if abs(self.wheel_radius) > 1e-6 else 0.0

        left_mul = right_mul = 1.0
        if abs(vx) > 1e-4 and abs(wz) > 1e-4:
            R = vx / wz
            left_mul = (R - self.track_width / 2.0) / R
            right_mul = (R + self.track_width / 2.0) / R

        targets_wheel = {
            "left_wheel_joint": omega * left_mul,
            "right_wheel_joint": omega * right_mul,
            "wheel_back_left_joint": omega * left_mul,
            "wheel_back_right_joint": omega * right_mul,
        }

        # WHEEL PID
        cmd = Float64MultiArray()
        for j in self.wheel_joints:
            error = targets_wheel[j] - self.wheel_vel[j]
            u = self.pid_wheel[j].compute(error, self.dt)
            cmd.data.append(u)

        self.wheel_pub.publish(cmd)


# MAIN
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToJointsPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
