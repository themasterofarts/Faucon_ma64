#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from sensor_msgs.msg import NavSatFix
from robot_localization.srv import SetDatum
from geographic_msgs.msg import GeoPose


LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


def _is_valid_fix(msg: NavSatFix) -> bool:
    if msg.status.status < 0:
        return False
    if math.isnan(msg.latitude) or math.isnan(msg.longitude):
        return False
    if abs(msg.latitude) < 1e-9 and abs(msg.longitude) < 1e-9:
        return False
    return True


class DatumManager(Node):
    """
    Capte le premier fix GNSS valide et l'envoie comme datum de reference.

    - Appelle le service /datum (robot_localization/srv/SetDatum)
      -> fixe l'origine de navsat_transform_node
    - Publie /gnss/datum (sensor_msgs/NavSatFix, TRANSIENT_LOCAL)
      -> consomme par follower.py et tout autre noeud qui a besoin
         de l'origine de la frame locale
    """

    def __init__(self):
        super().__init__("datum_manager")

        self._datum_fix: NavSatFix | None = None
        self._datum_sent = False

        # Publisher TRANSIENT_LOCAL pour les autres noeuds (follower.py, etc.)
        self._pub_fix = self.create_publisher(NavSatFix, "/gnss/datum", LATCHED_QOS)

        # Client vers le service /datum de navsat_transform_node
        self._srv_client = self.create_client(SetDatum, "/datum")

        # Subscriber GNSS
        self._sub = self.create_subscription(
            NavSatFix, "/gnss/fix", self._on_fix, 10
        )

        # Timer de retry : essaie d'envoyer le datum au service toutes les 0.5 s
        # tant que le service n'est pas disponible ou que le datum n'a pas ete envoye
        self._retry_timer = self.create_timer(0.5, self._try_send_datum)

        self.get_logger().info("datum_manager demarre - attente du premier fix GNSS valide...")

    # ------------------------------------------------------------------
    # GNSS callback - ne s'execute qu'une fois
    # ------------------------------------------------------------------

    def _on_fix(self, msg: NavSatFix) -> None:
        if self._datum_fix is not None:
            return
        if not _is_valid_fix(msg):
            return

        self._datum_fix = msg
        self.destroy_subscription(self._sub)

        alt = msg.altitude if not math.isnan(msg.altitude) else 0.0

        # Publication immediate sur /gnss/datum pour les autres noeuds
        datum_fix = NavSatFix()
        datum_fix.header.stamp = self.get_clock().now().to_msg()
        datum_fix.header.frame_id = "map"
        datum_fix.latitude = msg.latitude
        datum_fix.longitude = msg.longitude
        datum_fix.altitude = alt
        datum_fix.status = msg.status
        self._pub_fix.publish(datum_fix)

        self.get_logger().info(
            f"Fix GNSS capte -> lat={msg.latitude:.9f}, lon={msg.longitude:.9f}, alt={alt:.2f}. "
            "Envoi au service /datum en cours..."
        )

    # ------------------------------------------------------------------
    # Timer - appelle le service des qu'il est disponible
    # ------------------------------------------------------------------

    def _try_send_datum(self) -> None:
        if self._datum_sent:
            self._retry_timer.cancel()
            return

        if self._datum_fix is None:
            return  # pas encore de fix

        if not self._srv_client.service_is_ready():
            self.get_logger().info(
                "Service /datum pas encore disponible, nouvelle tentative dans 0.5 s...",
                throttle_duration_sec=5.0,
            )
            return

        self._retry_timer.cancel()
        self._send_datum_request()

    def _send_datum_request(self) -> None:
        msg = self._datum_fix
        alt = msg.altitude if not math.isnan(msg.altitude) else 0.0

        req = SetDatum.Request()
        req.geo_pose = GeoPose()
        req.geo_pose.position.latitude = msg.latitude
        req.geo_pose.position.longitude = msg.longitude
        req.geo_pose.position.altitude = alt
        req.geo_pose.orientation.w = 1.0

        future = self._srv_client.call_async(req)
        future.add_done_callback(self._on_service_response)
        self.get_logger().info(
            f"Requete SetDatum envoyee -> lat={msg.latitude:.9f}, lon={msg.longitude:.9f}"
        )

    def _on_service_response(self, future) -> None:
        try:
            future.result()
            self._datum_sent = True
            self.get_logger().info("Service /datum acquitte - navsat_transform est ancre.")
        except Exception as e:
            self.get_logger().error(f"Erreur service /datum : {e}. Nouvelle tentative...")
            self._datum_sent = False
            self._retry_timer = self.create_timer(0.5, self._try_send_datum)


def main():
    rclpy.init()
    node = DatumManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
