"""Integration : securite en mode DiffDrive (twist_mux + diffdrive_safety reels).

En mode DiffDrive (robot mini, use_ros2_control:=false), /cmd_vel va directement
au plugin Gazebo, qui garde la derniere vitesse : diffdrive_safety doit forcer
une vitesse nulle sur /e_stop et quand les commandes s'arretent.
"""
import time

import pytest
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from harness import NodeProcess, Probe, publish_for, repo_path, ros_executable


def twist(vx):
    t = Twist()
    t.linear.x = float(vx)
    return t


def flag(value):
    b = Bool()
    b.data = value
    return b


@pytest.fixture(scope="module")
def chain():
    mux = NodeProcess([
        ros_executable("twist_mux", "twist_mux"), "--ros-args",
        "--params-file", repo_path("faucon_base_desc", "config", "twist_mux.yaml"),
        "-p", "use_stamped:=false", "-r", "cmd_vel_out:=/cmd_vel",
    ], "twist_mux_dd")
    safety = NodeProcess.python(repo_path("faucon_control", "script", "diffdrive_safety.py"), "diffdrive_safety")
    probe = Probe("it_probe_dd")
    probe.cmd = []  # (temps, vx) recus sur /cmd_vel (ce que garderait le DiffDrive)
    probe.create_subscription(Twist, "/cmd_vel", lambda m: probe.cmd.append((time.monotonic(), m.linear.x)), 50)
    probe.nav = probe.create_publisher(Twist, "/cmd_vel_nav2", 10)
    probe.estop = probe.create_publisher(Bool, "/e_stop", 10)
    ready = Probe.wait_until(lambda: probe.count_subscribers("/cmd_vel_nav2") > 0
                             and probe.count_subscribers("/e_stop") >= 2
                             and probe.count_publishers("/cmd_vel") >= 2, timeout=20)
    assert ready, "chaine non connectee :\n" + mux.log()[-2000:] + safety.log()[-2000:]
    yield probe
    probe.estop.publish(flag(False))
    probe.close()
    assert safety.stop() == 0, "diffdrive_safety ne s'arrete pas proprement :\n" + safety.log()[-2000:]
    mux.stop()


def held(probe):
    """Vitesse que garderait le DiffDrive Gazebo : derniere valeur recue."""
    return probe.cmd[-1][1] if probe.cmd else None


def test_watchdog_forces_zero_when_commands_stop(chain):
    publish_for(chain.nav, lambda: twist(0.3), 0.6, 20)
    assert held(chain) == pytest.approx(0.3)
    t_last = time.monotonic()
    assert Probe.wait_until(lambda: held(chain) == 0.0, timeout=2.0)
    t_zero = next(t for t, v in chain.cmd if t > t_last and v == 0.0)
    delay = t_zero - t_last  # cmd_timeout = 0.5 s ; borne haute large (charge CI)
    assert 0.3 < delay < 2.0, f"vitesse forcee a zero {delay:.2f} s apres la derniere commande (attendu ~0.5 s)"


def test_e_stop_forces_zero_while_nav2_insists(chain):
    time.sleep(0.8)
    end = time.monotonic() + 0.5
    while time.monotonic() < end:
        chain.nav.publish(twist(0.3))
        time.sleep(0.05)
    chain.estop.publish(flag(True))
    t_estop = time.monotonic()
    end = time.monotonic() + 1.5
    while time.monotonic() < end:  # Nav2 continue de publier
        chain.nav.publish(twist(0.3))
        time.sleep(0.05)
    after = [v for t, v in chain.cmd if t > t_estop + 0.2]
    assert after and all(v == 0.0 for v in after), "une commande non nulle est passee pendant l'arret d'urgence"
    assert held(chain) == 0.0

    chain.estop.publish(flag(False))
    time.sleep(0.3)
    publish_for(chain.nav, lambda: twist(0.3), 0.6, 20)
    assert held(chain) == pytest.approx(0.3)
