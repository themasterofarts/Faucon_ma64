#!/usr/bin/env python3
from __future__ import annotations

import os
import math
import yaml
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from geographic_msgs.msg import GeoPoint

from robot_localization.srv import FromLLArray
from nav2_msgs.action import FollowPath


LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


# ----------------------------
# Utils
# ----------------------------

def expand_path(p: str) -> str:
    return os.path.expanduser(p)


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


@dataclass(frozen=True)
class GpsWaypoint:
    lat: float
    lon: float
    yaw: float


def load_waypoints_yaml(file_path: str) -> List[GpsWaypoint]:
    file_path = expand_path(file_path)
    if not os.path.isfile(file_path):
        raise FileNotFoundError(f"YAML introuvable: {file_path}")

    with open(file_path, "r") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict) or "waypoints" not in data:
        raise KeyError("Le YAML doit contenir une cle racine 'waypoints'")

    wps = data["waypoints"]
    if not isinstance(wps, list) or len(wps) == 0:
        raise ValueError("'waypoints' doit etre une liste non vide")

    out: List[GpsWaypoint] = []
    for i, w in enumerate(wps):
        try:
            out.append(GpsWaypoint(
                lat=float(w["latitude"]),
                lon=float(w["longitude"]),
                yaw=float(w.get("yaw", 0.0)),
            ))
        except Exception as e:
            raise ValueError(f"Waypoint #{i} invalide: {w}") from e

    return out


# ----------------------------
# ROS2 Node
# ----------------------------

class GpsPathSender(Node):
    def __init__(self):
        super().__init__("gps_path_sender")

        # Params
        default_yaml_path = os.path.join(
            get_package_share_directory("faucon_navigation"),
            "config",
            "generated_trajectory.yaml",
        )
        self.declare_parameter("yaml_path", default_yaml_path)
        self.declare_parameter("action_name", "/follow_path")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("wait_action_sec", 10.0)

        self.yaml_path      = self.get_parameter("yaml_path").get_parameter_value().string_value
        self.action_name    = self.get_parameter("action_name").get_parameter_value().string_value
        self.frame_id       = self.get_parameter("frame_id").get_parameter_value().string_value
        self.wait_action_sec = self.get_parameter("wait_action_sec").get_parameter_value().double_value

        self._gps_waypoints = load_waypoints_yaml(self.yaml_path)
        self.get_logger().info(
            f"Charge {len(self._gps_waypoints)} waypoints depuis {expand_path(self.yaml_path)}"
        )

        self._sent = False

        # Client de conversion GPS -> ENU fourni par navsat_transform_node
        self._fromll_client = self.create_client(FromLLArray, "/fromLLArray")

        # Action client Nav2
        self._nav_client = ActionClient(self, FollowPath, self.action_name)

        self._pub_debug = self.create_publisher(Path, "debug_path", 10)

        # Attend le datum publie par datum_manager (TRANSIENT_LOCAL :
        # recu meme si datum_manager a publie avant ce noeud)
        self._datum_sub = self.create_subscription(
            NavSatFix, "/gnss/datum", self._on_datum, LATCHED_QOS
        )
        self.get_logger().info("En attente du datum GNSS sur '/gnss/datum'...")

    # ------------------------------------------------------------------
    # Datum recu -> conversion des waypoints via /fromLLArray
    # ------------------------------------------------------------------

    def _on_datum(self, _: NavSatFix) -> None:
        if self._sent:
            return

        self.destroy_subscription(self._datum_sub)
        self.get_logger().info("Datum recu — conversion des waypoints via /fromLLArray...")
        self._call_fromll()

    def _call_fromll(self) -> None:
        if not self._fromll_client.service_is_ready():
            self.get_logger().info(
                "Service /fromLLArray pas encore disponible, nouvelle tentative dans 0.5 s...",
                throttle_duration_sec=5.0,
            )
            self.create_timer(0.5, self._retry_fromll)
            return

        self._send_fromll_request()

    def _retry_fromll(self) -> None:
        if self._sent or not self._fromll_client.service_is_ready():
            return
        self._send_fromll_request()

    def _send_fromll_request(self) -> None:
        req = FromLLArray.Request()
        req.ll_points = [
            GeoPoint(latitude=wp.lat, longitude=wp.lon, altitude=0.0)
            for wp in self._gps_waypoints
        ]

        future = self._fromll_client.call_async(req)
        future.add_done_callback(self._on_fromll_response)
        self.get_logger().info(
            f"Requete /fromLLArray envoyee ({len(req.ll_points)} points)..."
        )

    # ------------------------------------------------------------------
    # Reponse /fromLLArray -> construction du Path -> envoi a Nav2
    # ------------------------------------------------------------------

    def _on_fromll_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Erreur /fromLLArray : {e}")
            return

        map_points: List[Point] = response.map_points
        if len(map_points) != len(self._gps_waypoints):
            self.get_logger().error(
                f"Nb points retournes ({len(map_points)}) != nb waypoints ({len(self._gps_waypoints)})"
            )
            return

        path_msg = self._build_path(map_points)
        self.get_logger().info(
            f"Path construit: {len(path_msg.poses)} poses. "
            f"Premier point ENU: x={path_msg.poses[0].pose.position.x:.3f}, "
            f"y={path_msg.poses[0].pose.position.y:.3f}"
        )
        self._send_follow_path(path_msg)
        self._sent = True

    def _build_path(self, map_points: List[Point]) -> Path:
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        now = path.header.stamp

        for pt, wp in zip(map_points, self._gps_waypoints):
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = now
            ps.pose.position.x = pt.x
            ps.pose.position.y = pt.y
            ps.pose.position.z = 0.0
            ps.pose.orientation = yaw_to_quat(wp.yaw)
            path.poses.append(ps)

        return path

    # ------------------------------------------------------------------
    # Envoi de l'action FollowPath a Nav2
    # ------------------------------------------------------------------

    def _send_follow_path(self, path: Path) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=self.wait_action_sec):
            self.get_logger().error(
                f"Action server '{self.action_name}' indisponible apres {self.wait_action_sec}s."
            )
            return

        goal = FollowPath.Goal()
        goal.path = path
        self._pub_debug.publish(path)

        self.get_logger().info(f"Envoi du goal FollowPath vers '{self.action_name}'...")
        send_future = self._nav_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, _) -> None:
        self.get_logger().debug("Feedback FollowPath recu.")

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal FollowPath refuse.")
            return
        self.get_logger().info("Goal FollowPath accepte. Attente du resultat...")
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, _) -> None:
        self.get_logger().info("FollowPath termine.")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = GpsPathSender()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
