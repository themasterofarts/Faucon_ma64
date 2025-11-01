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
        self.declare_parameter("wheel_radius", 0.35)  # Wheel radius in meters
        self.declare_parameter(
            "wheel_base", 1.4
        )  # Distance between front and rear axles
        self.declare_parameter("track_width", 1.2)  # Left-right wheel distance
        self.declare_parameter("max_steer", math.pi / 3)  # Steering limit (rad, 60 deg)
        self.declare_parameter("steer_gain", 1.0)  # Gain for steering angle
        self.declare_parameter("wheel_gain", 1.0)  # Gain for wheel velocity

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_base = self.get_parameter("wheel_base").value
        self.track_width = self.get_parameter("track_width").value
        self.max_steer = self.get_parameter("max_steer").value
        self.steer_gain = self.get_parameter("steer_gain").value
        self.wheel_gain = self.get_parameter("wheel_gain").value

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

        # Compute steering angle using bicycle model
        if abs(wz) < EPS_WZ:
           
            steer_angle = 0.0
        elif abs(vx) > EPS_VX:
            
            denom = max(abs(vx), EPS_VX)
            steer_angle = math.atan2(self.wheel_base * wz, denom)
        else:
           
            steer_angle = math.copysign(self.max_steer * 0.8, wz)

        # clamp + gains
        steer_angle = max(-self.max_steer, min(self.max_steer, steer_angle)) * self.steer_gain

        # 4WS: AR opposé
        front_steer = steer_angle
        rear_steer  = -steer_angle

        
        jt_steer = JointTrajectory()
        jt_steer.joint_names = self.steer_joints
        pt = JointTrajectoryPoint()
        pt.positions = [front_steer, front_steer, rear_steer, rear_steer]
        pt.time_from_start = Duration(seconds=0.1).to_msg()
        jt_steer.points = [pt]
        self.steering_pub.publish(jt_steer)

        # Compute wheel angular velocities: wheel_omega = vx / wheel_radius
        wheel_omega = (vx / self.wheel_radius) * self.wheel_gain if abs(self.wheel_radius) > 1e-6 else 0.0
        left_multiplier = right_multiplier = 1.0
        if abs(wz) > 1e-5 and abs(vx) > 1e-5:
            R_center = vx / wz
            R_left  = R_center - (self.track_width / 2.0)
            R_right = R_center + (self.track_width / 2.0)
            left_multiplier  = R_left / R_center
            right_multiplier = R_right / R_center

        fl = wheel_omega * left_multiplier
        fr = wheel_omega * right_multiplier
        rl = wheel_omega * left_multiplier
        rr = wheel_omega * right_multiplier

        wheel_msg = Float64MultiArray()
        wheel_msg.data = [fl, fr, rl, rr]
        self.wheel_pub.publish(wheel_msg)



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