#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry



class PoseRobot(Node):


    def __init__(self):
        super().__init__("position_robot")

        self.get_logger().info("Node position robot started")

        # Topic de pose : les EKF de robot_localization publient odometry/local
        # (repere odom) et odometry/global (repere map) ; odometry/filtered
        # n'existe plus (remappe dans dual_ekf_navsat.launch.py).
        self.declare_parameter("odom_topic", "odometry/global")
        odom_topic = self.get_parameter("odom_topic").value

        #### s'aboonner pour récupéré la pose du robot

        self.sub = self.create_subscription(Odometry, odom_topic, self.pose_robot_callback, 10)




    def pose_robot_callback(self, msg) :

        ### recuperation de la pose local du robot

        x_robot = msg.pose.pose.position.x
        y_robot = msg.pose.pose.position.y


        ### rotation autour de z (yaw), sans dependance a tf_transformations
        q = msg.pose.pose.orientation
        yaw_local = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        self.get_logger().info( f"  position local  [x_robot= {x_robot:.3f}m ,  y_robot= {y_robot:.3f}m ],yaw_local= {yaw_local:.3f} rad ")


def main():
    rclpy.init()
    node =PoseRobot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
