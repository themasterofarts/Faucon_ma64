"""Utilitaires ROS 2 communs aux noeuds Python de Faucon.

Avant ce module, ces fonctions etaient copiees dans plusieurs scripts
(faucon_control, faucon_localisation, faucon_navigation, faucon_perception).
Conventions : REP-103 (ENU), yaw en radians autour de z.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# QoS "latched" : le dernier message est conserve et livre aux abonnes qui se
# connectent apres la publication (datum GNSS, chemin de mission...).
LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


def yaw_to_quaternion(yaw: float):
    """Quaternion geometry_msgs/Quaternion d'une rotation de `yaw` rad autour de z."""
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(q) -> float:
    """Yaw (rad, dans [-pi, pi]) d'un quaternion (objet avec attributs x, y, z, w)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def spin_until_shutdown(node) -> None:
    """Fait tourner `node` jusqu'a l'arret, puis le detruit et arrete rclpy.

    Ctrl+C (SIGINT) et l'arret par ros2 launch (SIGTERM) terminent le noeud
    silencieusement, sans trace KeyboardInterrupt / ExternalShutdownException.
    """
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
