"""Tests de la cinematique 4WS de control_4ws.py (necessite rclpy).

Pour chaque commande (vx, wz), on reconstruit la vitesse reelle du robot a
partir des braquages et vitesses de roues publies (moindres carres) et on la
compare a la commande. Couvre la marche arriere et la rotation sur place.
"""
import importlib.util
import math
import os

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import Twist  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

SCRIPT = os.path.join(os.path.dirname(__file__), "..", "script", "control_4ws.py")


@pytest.fixture(scope="module")
def module():
    spec = importlib.util.spec_from_file_location("control_4ws", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    rclpy.init()
    yield mod
    rclpy.try_shutdown()


@pytest.fixture
def node(module):
    n = module.CmdVelToJoints()
    n.max_steer_rate = 1e6  # pas de limitation de vitesse de braquage pour le test
    n.out = {}
    n.steering_pub.publish = lambda m: n.out.__setitem__("steer", list(m.points[0].positions))
    n.wheel_pub.publish = lambda m: n.out.__setitem__("wheels", list(m.data))
    yield n
    n.destroy_node()


def _command(node, vx, wz):
    t = Twist()
    t.linear.x, t.angular.z = vx, wz
    for _ in range(3):
        node.cmd_vel_cb(t)


def _body_velocity(node):
    half_l, half_w = node.wheel_base / 2.0, node.track_width / 2.0
    positions = [(half_l, half_w), (half_l, -half_w), (-half_l, half_w), (-half_l, -half_w)]
    a, b = [], []
    for (x, y), steer, w in zip(positions, node.out["steer"], node.out["wheels"]):
        v = w * node.wheel_radius
        a += [[1, 0, -y], [0, 1, x]]
        b += [v * math.cos(steer), v * math.sin(steer)]
    return np.linalg.lstsq(np.array(a), np.array(b), rcond=None)[0]


@pytest.mark.parametrize("vx, wz", [
    (0.5, 0.0),     # avant
    (-0.5, 0.0),    # arriere (tournait sur place avant correction)
    (0.4, 0.3),     # avant + virage
    (-0.4, 0.3),    # arriere + virage
    (0.0, 0.5),     # rotation sur place (avancait avant correction)
])
def test_kinematics(node, vx, wz):
    _command(node, vx, wz)
    rvx, rvy, rwz = _body_velocity(node)
    assert rvx == pytest.approx(vx, abs=0.02)
    assert rvy == pytest.approx(0.0, abs=0.02)
    assert rwz == pytest.approx(wz, abs=0.02)
    assert all(abs(s) <= node.max_steer + 1e-9 for s in node.out["steer"])


def test_reverse_keeps_wheels_straight(node):
    _command(node, -0.5, 0.0)
    assert all(abs(s) < 1e-6 for s in node.out["steer"])
    assert all(w < 0 for w in node.out["wheels"])


def test_e_stop_zeroes_wheels_and_blocks_cmd_vel(node):
    _command(node, 0.5, 0.0)
    assert any(abs(w) > 0 for w in node.out["wheels"])
    m = Bool()
    m.data = True
    node.e_stop_cb(m)
    assert all(w == 0.0 for w in node.out["wheels"])
    _command(node, 0.5, 0.0)  # ignore tant que l'arret d'urgence est actif
    assert all(w == 0.0 for w in node.out["wheels"])
    m.data = False
    node.e_stop_cb(m)
    _command(node, 0.5, 0.0)
    assert any(abs(w) > 0 for w in node.out["wheels"])
