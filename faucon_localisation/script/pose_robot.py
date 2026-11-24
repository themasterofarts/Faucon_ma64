#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from faucon_common.ros_utils import quaternion_to_yaw, spin_until_shutdown  # utilitaires partages (paquet faucon_common)



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
        yaw_local = quaternion_to_yaw(msg.pose.pose.orientation)

        self.get_logger().info( f"  position local  [x_robot= {x_robot:.3f}m ,  y_robot= {y_robot:.3f}m ],yaw_local= {yaw_local:.3f} rad ")


def main():
    rclpy.init()
    node =PoseRobot()
    spin_until_shutdown(node)


if __name__ == "__main__":
    main()
