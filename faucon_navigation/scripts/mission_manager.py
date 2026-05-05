#!/usr/bin/env python3
"""ROS2 node — mission lifecycle manager."""
from __future__ import annotations

import json
import math
import os
import sys
import time
import uuid
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from geographic_msgs.msg import GeoPoint

from robot_localization.srv import FromLLArray
from nav2_msgs.action import FollowPath

# mission_core.py lives in the same install dir as this script, so we can import it directly.
_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)
from mission_core import State, validate_waypoints, nearest_wp_index  # noqa: E402


LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class MissionManager(Node):
    """
    Gère le cycle de vie complet d'une mission autonome.

    Flux standard :
      IHM ─(load_path)──► LOADING ──► READY
      IHM ─(START)───────────────────────────► RUNNING ──► COMPLETED ──► IDLE
      IHM ─(PAUSE)───────────────────────────► PAUSED
      IHM ─(RESUME)──────────────────────────► RUNNING  (chemin tronqué)
      IHM ─(STOP)────────────────────────────► ABORTED ──► IDLE

    Topics consommés :
      /mission/load_path  (std_msgs/String)  — contenu YAML du chemin
      /mission/command    (std_msgs/String)  — START | STOP | PAUSE | RESUME
      /gnss/datum         (NavSatFix, latch) — localisation prête
      odometry/global     (Odometry)         — pose courante

    Topics produits :
      /mission/status     (std_msgs/String)  — JSON 5 Hz
      /mission/path       (nav_msgs/Path)    — chemin ENU complet
    """

    def __init__(self):
        super().__init__("mission_manager")

        # état
        self._state      = State.IDLE
        self._error_msg  = ""
        self._mission_id = ""

        # données chemin
        self._raw_wps:   List[dict]     = []
        self._full_path: Optional[Path] = None
        self._total_wp   = 0
        self._current_wp = 0

        # robot
        self._robot_pose: Optional[PoseStamped] = None
        self._datum_ready = False
        self._last_odom_t = 0.0

        # Nav2
        self._goal_handle = None
        self._nav_client  = ActionClient(self, FollowPath, "/follow_path")

        # conversion GPS→ENU
        self._fromll             = self.create_client(FromLLArray, "/fromLLArray")
        self._fromll_retry_timer = None

        # subscribers
        self.create_subscription(String,    "/mission/load_path", self._on_load_path, 10)
        self.create_subscription(String,    "/mission/command",   self._on_command,   10)
        self.create_subscription(NavSatFix, "/gnss/datum",        self._on_datum,     LATCHED_QOS)
        self.create_subscription(Odometry,  "odometry/global",    self._on_odom,      10)

        # publishers
        self._pub_status = self.create_publisher(String, "/mission/status", 10)
        self._pub_path   = self.create_publisher(Path, "/mission/path", LATCHED_QOS)

        # timers
        self.create_timer(0.2, self._publish_status)  # 5 Hz
        self.create_timer(2.0, self._watchdog)

        self.get_logger().info("mission_manager prêt - état: IDLE")


    def _on_datum(self, _: NavSatFix) -> None:
        if not self._datum_ready:
            self._datum_ready = True
            self.get_logger().info("Datum GNSS reçu - conversions GPS→ENU disponibles.")

    def _on_odom(self, msg: Odometry) -> None:
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose   = msg.pose.pose
        self._robot_pose  = ps
        self._last_odom_t = time.monotonic()

        if self._state == State.RUNNING and self._full_path:
            positions = [(p.pose.position.x, p.pose.position.y) for p in self._full_path.poses]
            idx = nearest_wp_index(positions, ps.pose.position.x, ps.pose.position.y)
            if idx > self._current_wp:
                self._current_wp = idx

    def _on_load_path(self, msg: String) -> None:
        if self._state == State.RUNNING:
            self.get_logger().warn("Mission en cours - envoyez STOP avant de charger un nouveau chemin.")
            return
        if self._state == State.PAUSED:
            self._cancel_goal()
        self._begin_loading(msg.data)

    def _on_command(self, msg: String) -> None:
        cmd = msg.data.strip().upper()
        {
            "START":  self._cmd_start,
            "STOP":   self._cmd_stop,
            "PAUSE":  self._cmd_pause,
            "RESUME": self._cmd_resume,
        }.get(cmd, lambda: self.get_logger().warn(f"Commande inconnue: '{cmd}'"))()

    # -------------------------------------------------------------------------
    # Commandes
    # -------------------------------------------------------------------------

    def _cmd_start(self) -> None:
        if self._state != State.READY:
            self.get_logger().warn(f"START ignoré — état: {self._state}")
            return
        self._current_wp = 0
        self._send_path(self._full_path)

    def _cmd_stop(self) -> None:
        if self._state not in (State.RUNNING, State.PAUSED, State.READY, State.ERROR):
            self.get_logger().warn(f"STOP ignoré — état: {self._state}")
            return
        self._cancel_goal()
        self._transition(State.ABORTED)
        self._reset()
        self._transition(State.IDLE)

    def _cmd_pause(self) -> None:
        if self._state != State.RUNNING:
            self.get_logger().warn(f"PAUSE ignoré — état: {self._state}")
            return
        self._cancel_goal()
        self._transition(State.PAUSED)
        self.get_logger().info(f"[{self._mission_id}] Pause au WP {self._current_wp}/{self._total_wp}.")

    def _cmd_resume(self) -> None:
        if self._state != State.PAUSED:
            self.get_logger().warn(f"RESUME ignoré — état: {self._state}")
            return
        self._send_path(self._build_resume_path())

    # -------------------------------------------------------------------------
    # Chargement & conversion GPS→ENU
    # -------------------------------------------------------------------------

    def _begin_loading(self, yaml_str: str) -> None:
        self._transition(State.LOADING)
        try:
            wps = validate_waypoints(yaml_str)
        except ValueError as e:
            return self._set_error(str(e))

        if not self._datum_ready:
            return self._set_error("Datum GNSS non disponible — localisation non initialisée")

        self._raw_wps    = wps
        self._total_wp   = len(wps)
        self._current_wp = 0
        self._mission_id = uuid.uuid4().hex[:8].upper()
        self.get_logger().info(
            f"[{self._mission_id}] {self._total_wp} waypoints validés. Conversion GPS→ENU..."
        )
        self._start_conversion()

    def _start_conversion(self) -> None:
        if self._fromll.service_is_ready():
            self._do_fromll()
        else:
            self.get_logger().info("Attente du service /fromLLArray...")
            self._fromll_retry_timer = self.create_timer(1.0, self._retry_fromll)

    def _retry_fromll(self) -> None:
        if self._state != State.LOADING:
            self._cancel_timer(self._fromll_retry_timer)
            return
        if self._fromll.service_is_ready():
            self._cancel_timer(self._fromll_retry_timer)
            self._do_fromll()

    def _do_fromll(self) -> None:
        req = FromLLArray.Request()
        req.ll_points = [
            GeoPoint(
                latitude=float(w["latitude"]),
                longitude=float(w["longitude"]),
                altitude=float(w.get("altitude", 0.0)),
            )
            for w in self._raw_wps
        ]
        self._fromll.call_async(req).add_done_callback(self._on_fromll_response)

    def _on_fromll_response(self, future) -> None:
        try:
            resp = future.result()
        except Exception as e:
            return self._set_error(f"Erreur /fromLLArray: {e}")

        pts: List[Point] = resp.map_points
        if len(pts) != self._total_wp:
            return self._set_error(
                f"/fromLLArray: {len(pts)} points reçus pour {self._total_wp} attendus"
            )

        self._full_path = self._make_path(pts)
        self._pub_path.publish(self._full_path)
        self._transition(State.READY)
        p0 = self._full_path.poses[0].pose.position
        self.get_logger().info(
            f"[{self._mission_id}] Chemin ENU prêt — {self._total_wp} WP. "
            f"Origine: x={p0.x:.2f}, y={p0.y:.2f}. Envoyez START."
        )

    def _make_path(self, pts: List[Point]) -> Path:
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp    = self.get_clock().now().to_msg()
        now = path.header.stamp
        for pt, wp in zip(pts, self._raw_wps):
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.header.stamp    = now
            ps.pose.position.x = pt.x
            ps.pose.position.y = pt.y
            ps.pose.position.z = 0.0
            yaw = float(wp.get("yaw", 0.0))
            q   = Quaternion()
            q.z = math.sin(yaw * 0.5)
            q.w = math.cos(yaw * 0.5)
            ps.pose.orientation = q
            path.poses.append(ps)
        return path

    # -------------------------------------------------------------------------
    # Exécution Nav2
    # -------------------------------------------------------------------------

    def _send_path(self, path: Optional[Path]) -> None:
        if not path or not path.poses:
            return self._set_error("Chemin vide — impossible d'exécuter")
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            return self._set_error("Serveur d'action /follow_path indisponible (timeout 5 s)")

        goal = FollowPath.Goal()
        goal.path = path
        self._transition(State.RUNNING)
        self.get_logger().info(f"[{self._mission_id}] FollowPath envoyé — {len(path.poses)} poses.")
        self._nav_client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        ).add_done_callback(self._on_goal_accepted)

    def _on_feedback(self, _) -> None:
        pass

    def _on_goal_accepted(self, future) -> None:
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            return self._set_error("Goal FollowPath refusé par Nav2")
        self.get_logger().info(f"[{self._mission_id}] Goal accepté par Nav2.")
        self._goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        if self._state != State.RUNNING:
            return  # annulé volontairement (PAUSE / STOP)
        self._goal_handle = None
        try:
            status = future.result().status
        except Exception:
            status = GoalStatus.STATUS_ABORTED

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._transition(State.COMPLETED)
            self.get_logger().info(f"[{self._mission_id}] Mission terminée avec succès ✓")
            self.create_timer(3.0, self._auto_idle)
        else:
            self._set_error(f"Nav2 a terminé avec statut: {status}")

    def _auto_idle(self) -> None:
        if self._state == State.COMPLETED:
            self._reset()
            self._transition(State.IDLE)

    def _cancel_goal(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    # -------------------------------------------------------------------------
    # Reprise (troncature du chemin)
    # -------------------------------------------------------------------------

    def _build_resume_path(self) -> Path:
        resume_idx = max(0, min(self._current_wp, len(self._full_path.poses) - 1))
        if self._robot_pose is None:
            self.get_logger().warn("Reprise depuis le début (pose robot inconnue).")
            resume_idx = 0

        truncated = Path()
        truncated.header       = self._full_path.header
        truncated.header.stamp = self.get_clock().now().to_msg()
        truncated.poses        = self._full_path.poses[resume_idx:]

        if self._robot_pose:
            rx = self._robot_pose.pose.position.x
            ry = self._robot_pose.pose.position.y
            self.get_logger().info(
                f"[{self._mission_id}] Reprise depuis WP {resume_idx}/{self._total_wp} "
                f"| robot @ ({rx:.2f}, {ry:.2f}) | {len(truncated.poses)} poses restantes."
            )
        return truncated


    def _watchdog(self) -> None:
        if self._state == State.RUNNING and self._last_odom_t > 0.0:
            age = time.monotonic() - self._last_odom_t
            if age > 5.0:
                self.get_logger().warn(
                    f"[{self._mission_id}] Odométrie absente depuis {age:.1f} s — "
                    "vérifiez la localisation."
                )

    def _publish_status(self) -> None:
        progress = (self._current_wp / self._total_wp) if self._total_wp > 0 else 0.0
        msg      = String()
        msg.data = json.dumps({
            "state":      self._state.value,
            "mission_id": self._mission_id,
            "progress":   round(progress, 3),
            "current_wp": self._current_wp,
            "total_wp":   self._total_wp,
            "error":      self._error_msg,
        })
        self._pub_status.publish(msg)

    def _transition(self, new: State) -> None:
        if new != State.ERROR:
            self._error_msg = ""
        self.get_logger().info(f"[mission] {self._state.value} → {new.value}")
        self._state = new

    def _set_error(self, msg: str) -> None:
        self.get_logger().error(f"[mission] {msg}")
        self._error_msg = msg
        self._cancel_goal()
        self._state = State.ERROR

    def _reset(self) -> None:
        self._full_path   = None
        self._raw_wps     = []
        self._total_wp    = 0
        self._current_wp  = 0
        self._goal_handle = None
        self._mission_id  = ""
        self._error_msg   = ""

    @staticmethod
    def _cancel_timer(timer) -> None:
        if timer is not None:
            timer.cancel()


def main():
    rclpy.init()
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
