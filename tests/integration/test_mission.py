"""Integration : cycle de vie d'une mission (mission_manager + datum_manager reels).

Le test remplace Nav2 et navsat_transform par des faux, controles par le test :
  - /datum (SetDatum) et /fromLLArray (FromLLArray) : navsat_transform factice
  - /follow_path (FollowPath) : serveur d'action Nav2 factice, qui peut reussir
    apres un delai ou attendre une annulation
Le test pilote la mission comme l'IHM : /mission/load_path, /mission/command,
/e_stop, et lit l'etat publie sur /mission/status (JSON).
"""
import json
import time

import pytest
from action_msgs.msg import GoalStatus  # noqa: F401  (documente les statuts attendus)
from geometry_msgs.msg import Point
from nav2_msgs.action import FollowPath
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from robot_localization.srv import FromLLArray, SetDatum
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String

from harness import NodeProcess, Probe, repo_path

PATH_YAML = """waypoints:
- {latitude: 43.89997, longitude: 3.19997, yaw: 1.57}
- {latitude: 43.90000, longitude: 3.19997, yaw: 1.57}
- {latitude: 43.90003, longitude: 3.19997, yaw: 1.57}
"""


class FakeNav(Probe):
    """navsat_transform + Nav2 factices, et commandes facon IHM."""

    def __init__(self):
        super().__init__("it_fake_nav")
        cg = ReentrantCallbackGroup()
        self.datum_requests = []
        self.goals = []            # chemins recus par /follow_path
        self.cancels = 0
        self.mode = "hold"         # "hold" : attend l'annulation ; "succeed" : reussit apres 1 s
        self.closing = False
        self.create_service(SetDatum, "/datum", self._on_datum, callback_group=cg)
        self.create_service(FromLLArray, "/fromLLArray", self._on_fromll, callback_group=cg)
        ActionServer(self, FollowPath, "/follow_path", execute_callback=self._execute,
                     goal_callback=lambda g: GoalResponse.ACCEPT,
                     cancel_callback=self._on_cancel, callback_group=cg)
        self.status = None
        self.create_subscription(String, "/mission/status", self._on_status, 10)
        self.fix = self.create_publisher(NavSatFix, "/gnss/fix", 10)
        self.load = self.create_publisher(String, "/mission/load_path", 10)
        self.cmd = self.create_publisher(String, "/mission/command", 10)
        self.estop = self.create_publisher(Bool, "/e_stop", 10)

    # --- faux navsat_transform
    def _on_datum(self, req, res):
        self.datum_requests.append((req.geo_pose.position.latitude, req.geo_pose.position.longitude))
        return res

    def _on_fromll(self, req, res):
        res.map_points = [Point(x=float(i), y=0.0, z=0.0) for i in range(len(req.ll_points))]
        return res

    # --- faux Nav2
    def _on_cancel(self, goal_handle):
        self.cancels += 1
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        self.goals.append(goal_handle.request.path)
        start = time.monotonic()
        while True:
            if self.closing:  # fermeture du test : on termine le goal avant de detruire le noeud
                goal_handle.abort()
                return FollowPath.Result()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowPath.Result()
            if self.mode == "succeed" and time.monotonic() - start > 1.0:
                goal_handle.succeed()
                return FollowPath.Result()
            time.sleep(0.02)

    # --- facon IHM
    def _on_status(self, msg):
        self.status = json.loads(msg.data)

    def state(self):
        return self.status["state"] if self.status else None

    def send(self, cmd):
        self.cmd.publish(String(data=cmd))

    def close(self):
        self.closing = True
        time.sleep(0.2)  # laisse les boucles d'execution des goals se terminer
        super().close()

    def wait_state(self, state, timeout=8.0):
        ok = Probe.wait_until(lambda: self.state() == state, timeout)
        assert ok, f"etat attendu {state}, obtenu {self.state()} ({self.status})"


@pytest.fixture(scope="module")
def mission():
    nav = FakeNav()
    datum = NodeProcess.python(repo_path("faucon_localisation", "script", "datum_manager.py"), "datum_manager")
    manager = NodeProcess.python(repo_path("faucon_navigation", "scripts", "mission_manager.py"), "mission_manager")
    assert Probe.wait_until(lambda: nav.state() == "IDLE", timeout=20), "mission_manager ne publie pas son etat :\n" + manager.log()[-2000:]
    # un fix sans position valide doit etre ignore (NB : sous Jazzy, le statut par
    # defaut d'un NavSatFix est STATUS_UNKNOWN = -2 ; le bridge Gazebo met STATUS_FIX)
    bad = NavSatFix()
    bad.latitude, bad.longitude = 43.0, 3.0
    bad.status.status = NavSatStatus.STATUS_NO_FIX
    Probe.wait_until(lambda: (nav.fix.publish(bad), False)[1], timeout=1.5, period=0.2)
    nav.rejected_fix_ok = not nav.datum_requests
    # premier fix GNSS valide -> datum_manager appelle /datum et publie /gnss/datum (latched)
    fix = NavSatFix()
    fix.latitude, fix.longitude, fix.altitude = 43.89997, 3.19997, 0.0
    fix.status.status = NavSatStatus.STATUS_FIX
    assert Probe.wait_until(lambda: (nav.fix.publish(fix), bool(nav.datum_requests))[1], timeout=10, period=0.2), \
        "datum_manager n'a pas appele /datum :\n" + datum.log()[-2000:]
    yield nav
    nav.estop.publish(Bool(data=False))
    nav.send("STOP")
    time.sleep(0.3)
    nav.close()
    assert manager.stop() == 0, "mission_manager ne s'arrete pas proprement :\n" + manager.log()[-2000:]
    assert datum.stop() == 0, "datum_manager ne s'arrete pas proprement :\n" + datum.log()[-2000:]


def reset(nav):
    """Revient a IDLE. Attend entre deux commandes : l'ordre d'arrivee n'est pas
    garanti entre deux topics differents (/mission/command et /mission/load_path)."""
    nav.estop.publish(Bool(data=False))
    if nav.state() not in ("IDLE", None):
        nav.send("STOP")
        nav.wait_state("IDLE")
    time.sleep(0.3)


def load_ready(nav):
    """Charge le chemin UNE fois et attend READY. (Republier en boucle relancerait
    un chargement apres READY, et un START tombant pendant ce LOADING serait refuse.)"""
    reset(nav)
    for _ in range(3):
        nav.load.publish(String(data=PATH_YAML))
        if Probe.wait_until(lambda: nav.state() == "READY", timeout=3.0):
            break
    assert nav.state() == "READY", f"chemin non charge : {nav.status}"
    assert nav.status["total_wp"] == 3
    time.sleep(0.3)


def test_datum_is_sent_from_first_valid_gnss_fix(mission):
    assert mission.rejected_fix_ok, "un fix sans position valide a ete utilise comme datum"
    assert mission.datum_requests[0] == pytest.approx((43.89997, 3.19997))
    assert len(mission.datum_requests) == 1  # un seul datum, meme si d'autres fix arrivent


def test_invalid_path_is_rejected(mission):
    reset(mission)
    mission.load.publish(String(data="trajectory: []"))
    mission.wait_state("ERROR")
    assert "waypoints" in mission.status["error"]


def test_nominal_mission_completes_then_returns_to_idle(mission):
    load_ready(mission)
    mission.mode = "succeed"
    n_goals = len(mission.goals)
    mission.send("START")
    mission.wait_state("RUNNING")
    assert Probe.wait_until(lambda: len(mission.goals) == n_goals + 1, timeout=5)
    assert len(mission.goals[-1].poses) == 3
    mission.wait_state("COMPLETED")
    mission.wait_state("IDLE", timeout=6)  # retour automatique a IDLE apres 3 s


def test_pause_then_resume_sends_a_new_goal_and_keeps_running(mission):
    load_ready(mission)
    mission.mode = "hold"
    n_goals, n_cancels = len(mission.goals), mission.cancels
    mission.send("START")
    mission.wait_state("RUNNING")
    assert Probe.wait_until(lambda: len(mission.goals) == n_goals + 1, timeout=5)

    mission.send("PAUSE")
    mission.wait_state("PAUSED")
    assert Probe.wait_until(lambda: mission.cancels == n_cancels + 1, timeout=5), "goal Nav2 non annule a la pause"

    mission.send("RESUME")
    mission.wait_state("RUNNING")
    assert Probe.wait_until(lambda: len(mission.goals) == n_goals + 2, timeout=5), "pas de nouveau goal a la reprise"
    time.sleep(1.0)  # le resultat CANCELED de l'ancien goal ne doit pas faire passer la mission en ERROR
    assert mission.state() == "RUNNING"


def test_e_stop_pauses_the_mission_and_cancels_the_goal(mission):
    load_ready(mission)
    mission.mode = "hold"
    n_goals, n_cancels = len(mission.goals), mission.cancels
    mission.send("START")
    mission.wait_state("RUNNING")
    assert Probe.wait_until(lambda: len(mission.goals) == n_goals + 1, timeout=5)

    mission.estop.publish(Bool(data=True))
    mission.wait_state("PAUSED")
    assert Probe.wait_until(lambda: mission.cancels == n_cancels + 1, timeout=5)


def test_stop_returns_to_idle_and_cancels_the_goal(mission):
    load_ready(mission)
    mission.mode = "hold"
    n_goals, n_cancels = len(mission.goals), mission.cancels
    mission.send("START")
    mission.wait_state("RUNNING")
    assert Probe.wait_until(lambda: len(mission.goals) == n_goals + 1, timeout=5)
    mission.send("STOP")
    mission.wait_state("IDLE")
    assert Probe.wait_until(lambda: mission.cancels == n_cancels + 1, timeout=5)
