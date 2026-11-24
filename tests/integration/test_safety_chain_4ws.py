"""Integration : chaine de commande du robot 4WS (twist_mux + control_4ws reels).

  /cmd_vel_nav2 (prio 10) --+
                            +--> twist_mux --> /cmd_vel --> control_4ws --> roues
  /cmd_vel_teleop (prio 100)+        ^                          ^
                     /e_stop --------+ (verrou, prio 255) ------+ (roues a zero)
"""
import time

import pytest
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64MultiArray

from harness import NodeProcess, Probe, publish_for, repo_path, ros_executable

WHEEL_RADIUS = 0.25  # control_4ws : vitesse roue (rad/s) = vx / rayon en ligne droite


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
    ], "twist_mux_4ws")
    ctrl = NodeProcess.python(repo_path("faucon_control", "script", "control_4ws.py"), "control_4ws")
    probe = Probe("it_probe_4ws")
    probe.wheels = []  # (temps, liste des vitesses roues)
    probe.sent = []    # (temps, vx) des commandes envoyees, pour le diagnostic
    probe.create_subscription(Float64MultiArray, "/velocity_controller/commands",
                              lambda m: probe.wheels.append((time.monotonic(), list(m.data))), 10)
    probe.nav = probe.create_publisher(Twist, "/cmd_vel_nav2", 10)
    probe.teleop = probe.create_publisher(Twist, "/cmd_vel_teleop", 10)
    probe.estop = probe.create_publisher(Bool, "/e_stop", 10)
    # attendre que la chaine soit connectee (twist_mux abonne aux entrees, control_4ws a /cmd_vel)
    ready = Probe.wait_until(lambda: probe.count_subscribers("/cmd_vel_nav2") > 0
                             and probe.count_subscribers("/cmd_vel_teleop") > 0
                             and probe.count_subscribers("/e_stop") >= 2
                             and probe.count_publishers("/velocity_controller/commands") > 0, timeout=20)
    assert ready, "chaine non connectee :\n" + mux.log()[-2000:] + ctrl.log()[-2000:]
    probe.ctrl_log = ctrl.log
    # Echauffement de bout en bout (decouverte DDS plus lente en CI) : des commandes
    # jusqu'a voir les roues tourner, ce qui prouve que nav2 -> twist_mux -> control_4ws
    # -> roues est connecte.
    warm = False
    end = time.monotonic() + 30
    while not warm and time.monotonic() < end:
        publish_for(probe.nav, lambda: twist(0.3), 0.3, 20)
        warm = bool(probe.wheels) and any(abs(w) > 0 for w in probe.wheels[-1][1])
    assert warm, "control_4ws ne recoit pas les commandes :\n" + ctrl.log()[-2000:]
    yield probe
    probe.estop.publish(flag(False))
    probe.close()
    assert ctrl.stop() == 0, "control_4ws ne s'arrete pas proprement :\n" + ctrl.log()[-2000:]
    mux.stop()


def last_wheels(probe):
    return probe.wheels[-1][1] if probe.wheels else None


def settle(probe):
    """Etat neutre entre deux tests : arret d'urgence rearme, roues arretees."""
    probe.estop.publish(flag(False))
    time.sleep(0.3)
    publish_for(probe.nav, lambda: twist(0.0), 0.3, 20)
    time.sleep(0.8)  # > timeout twist_mux (0.5 s)


def test_nav2_command_drives_wheels(chain):
    settle(chain)
    publish_for(chain.nav, lambda: twist(0.3), 1.0, 20)
    assert last_wheels(chain) == pytest.approx([0.3 / WHEEL_RADIUS] * 4, abs=1e-3)


def test_teleop_has_priority_over_nav2(chain):
    settle(chain)
    end = time.monotonic() + 1.0
    while time.monotonic() < end:  # Nav2 et teleop publient en meme temps
        chain.nav.publish(twist(0.3))
        chain.teleop.publish(twist(0.8))
        time.sleep(0.05)
    assert last_wheels(chain) == pytest.approx([0.8 / WHEEL_RADIUS] * 4, abs=1e-3)


def timeline(probe, t_ref, window=1.5):
    """Chronologie autour de t_ref (diagnostic en cas d'echec)."""
    ev = [(t - t_ref, f"envoi nav vx={v}") for t, v in probe.sent if abs(t - t_ref) < window]
    ev += [(t - t_ref, f"roues {[round(x, 2) for x in w]}") for t, w in probe.wheels if abs(t - t_ref) < window]
    lines = [f"  {dt:+.3f} s  {what}" for dt, what in sorted(ev)]
    return "\n".join(lines[-40:]) + "\n--- journal control_4ws ---\n" + probe.ctrl_log()[-1500:]


def test_watchdog_stops_wheels_when_commands_stop(chain):
    settle(chain)
    end = time.monotonic() + 0.6
    while time.monotonic() < end:
        chain.nav.publish(twist(0.3))
        chain.sent.append((time.monotonic(), 0.3))
        time.sleep(0.05)
    t_last_cmd = chain.sent[-1][0]
    assert any(abs(w) > 0 for w in last_wheels(chain))
    stopped = Probe.wait_until(lambda: all(w == 0.0 for w in last_wheels(chain)), timeout=2.0)
    assert stopped, "les roues ne sont pas arretees apres l'arret des commandes"
    t_zero = next(t for t, w in chain.wheels if t > t_last_cmd and all(v == 0.0 for v in w))
    # cmd_timeout = 0.5 s : pas d'arret premature ; borne haute large pour absorber la
    # charge d'une CI (latences de reception de la sonde)
    delay = t_zero - t_last_cmd
    assert 0.3 < delay < 2.0, (f"arret des roues {delay:.2f} s apres la derniere commande (attendu ~0.5 s)\n"
                               + timeline(chain, t_last_cmd))


def test_e_stop_blocks_nav2_and_teleop_until_rearmed(chain):
    settle(chain)
    publish_for(chain.nav, lambda: twist(0.3), 0.5, 20)
    assert any(abs(w) > 0 for w in last_wheels(chain))

    chain.estop.publish(flag(True))
    t_estop = time.monotonic()
    wall_estop = time.time()  # comparable aux horodatages du journal de control_4ws
    assert Probe.wait_until(lambda: all(w == 0.0 for w in last_wheels(chain)), timeout=0.5), \
        "roues non arretees dans les 0,5 s suivant l'arret d'urgence"
    delay = next(t for t, w in chain.wheels if t >= t_estop and all(v == 0.0 for v in w)) - t_estop
    if delay >= 0.5:
        import re
        logs = [(float(m.group(1)) - wall_estop, m.group(2)) for m in
                re.finditer(r"\[(\d+\.\d+)\] \[cmd_vel_based_4ws_control\]: (.*)", chain.ctrl_log())]
        detail = "\n".join(f"  {dt:+.3f} s  control_4ws: {txt}" for dt, txt in logs if abs(dt) < 3)
        raise AssertionError(f"roues arretees {delay:.2f} s apres l'arret d'urgence (attendu quasi immediat)\n"
                             f"journal control_4ws relatif a l'envoi de /e_stop :\n{detail}\n" + timeline(chain, t_estop))

    # Nav2 et teleop insistent : rien ne doit passer
    n_before = len(chain.wheels)
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        chain.nav.publish(twist(0.3))
        chain.teleop.publish(twist(0.8))
        time.sleep(0.05)
    assert all(all(v == 0.0 for v in w) for _, w in chain.wheels[n_before:])

    # rearmement : Nav2 reprend la main
    chain.estop.publish(flag(False))
    time.sleep(0.3)
    publish_for(chain.nav, lambda: twist(0.3), 0.8, 20)
    assert last_wheels(chain) == pytest.approx([0.3 / WHEEL_RADIUS] * 4, abs=1e-3)
