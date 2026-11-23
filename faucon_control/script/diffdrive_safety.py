#!/usr/bin/env python3
"""Securite /cmd_vel pour les modes DiffDrive Gazebo (robot mini, use_ros2_control:=false).

Dans ces modes, control_4ws n'est pas lance : /cmd_vel (sortie de twist_mux) est
transmis directement au plugin DiffDrive de Gazebo, qui conserve la DERNIERE
vitesse recue. Sans ce noeud :
  - un arret d'urgence (/e_stop) bloque bien twist_mux, mais le robot continue
    a la derniere vitesse ;
  - si Nav2 / l'IHM cessent de publier, le robot continue indefiniment.

Ce noeud :
  - publie une vitesse nulle sur /cmd_vel a 10 Hz tant que /e_stop est actif
    (twist_mux bloque toutes les autres sources : aucune commande concurrente) ;
  - publie une vitesse nulle si aucune commande n'a ete recue depuis cmd_timeout
    apres une commande non nulle (meme role que le watchdog de control_4ws).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


def _is_zero(msg: Twist, eps: float = 1e-6) -> bool:
    return all(
        abs(v) < eps
        for v in (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
    )


class DiffDriveSafety(Node):
    def __init__(self):
        super().__init__("diffdrive_safety")
        self.declare_parameter("cmd_timeout", 0.5)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        self.e_stop_active = False
        self.moving = False           # derniere commande recue non nulle
        self.last_cmd_rx_time = None

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Bool, "/e_stop", self._on_e_stop, 10)
        self.create_timer(0.1, self._on_timer)

        self.get_logger().info(
            f"diffdrive_safety actif (cmd_timeout={self.cmd_timeout:.2f} s, /e_stop surveille)"
        )

    def _publish_zero(self) -> None:
        self.cmd_pub.publish(Twist())
        self.moving = False

    def _on_cmd_vel(self, msg: Twist) -> None:
        # Nos propres zeros reviennent aussi ici : ils ne relancent pas le watchdog.
        if _is_zero(msg):
            self.moving = False
            return
        self.moving = True
        self.last_cmd_rx_time = self.get_clock().now()

    def _on_e_stop(self, msg: Bool) -> None:
        active = bool(msg.data)
        if active and not self.e_stop_active:
            self.get_logger().error("ARRET D'URGENCE actif : /cmd_vel force a zero.")
            self._publish_zero()
        elif not active and self.e_stop_active:
            self.get_logger().warn("Arret d'urgence rearme.")
        self.e_stop_active = active

    def _on_timer(self) -> None:
        if self.e_stop_active:
            self._publish_zero()
            return
        if not self.moving or self.last_cmd_rx_time is None or self.cmd_timeout <= 0.0:
            return
        age = (self.get_clock().now() - self.last_cmd_rx_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self.get_logger().warn(
                f"Aucune commande /cmd_vel depuis {age:.2f} s : vitesse forcee a zero."
            )
            self._publish_zero()


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
