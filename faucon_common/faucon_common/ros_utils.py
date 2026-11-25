"""Utilitaires ROS 2 communs aux noeuds Python de Faucon.

Avant ce module, ces fonctions etaient copiees dans plusieurs scripts
(faucon_control, faucon_localisation, faucon_navigation, faucon_perception).
Conventions : REP-103 (ENU), yaw en radians autour de z.
"""
from __future__ import annotations

import math
import signal
import threading

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
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


def spin_until_shutdown(node, poll_period: float = 0.1) -> None:
    """Fait tourner `node` jusqu'a l'arret, puis le detruit et arrete rclpy.

    Ctrl+C (SIGINT) et l'arret par ros2 launch (SIGTERM) terminent le noeud
    silencieusement (code de sortie 0).

    Les signaux ne levent pas d'exception : le gestionnaire installe ici leve
    seulement un drapeau, verifie entre deux iterations de l'executeur (au plus
    `poll_period` s). Avec rclpy.spin(), une interruption arrivant pendant la
    lecture d'un message (code C++) remontait parfois en RuntimeError au lieu de
    KeyboardInterrupt (noeuds a fort debit, ex. diffdrive_safety sur /cmd_vel),
    d'ou une trace et un code de sortie 1.
    """
    stop = threading.Event()

    def _request_stop(signum, frame):
        stop.set()

    previous = {}
    if threading.current_thread() is threading.main_thread():
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, _request_stop)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not stop.is_set():
            executor.spin_once(timeout_sec=poll_period)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        executor.shutdown(timeout_sec=1.0)
    node.destroy_node()
    rclpy.try_shutdown()
