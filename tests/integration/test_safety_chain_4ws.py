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

from harness import NodeProcess, Probe, publish_for, publish_until, repo_path, ros_executable, watchdog_ages

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


def wheels_are(probe, value):
    w = last_wheels(probe)
    return w is not None and all(abs(v - value) < 1e-3 for v in w)


def settle(probe):
    """Etat neutre entre deux tests : arret d'urgence rearme, roues arretees."""
    probe.estop.publish(flag(False))
    time.sleep(0.3)
    publish_for(probe.nav, lambda: twist(0.0), 0.3, 20)
    time.sleep(0.8)  # > timeout twist_mux (0.5 s)


# NB sur les temps : en CI (conteneur GitHub), un abonne DDS peut ponctuellement ne
# rien recevoir pendant plus de 0,5 s alors qu'un autre abonne au meme topic recoit
# tout (observe dans les journaux des noeuds). Les tests publient donc jusqu'a observer
# l'effet attendu, et le delai de securite est verifie d'apres l'age mesure par le noeud
# lui-meme depuis sa derniere reception, pas d'apres l'horloge du test.

def test_nav2_command_drives_wheels(chain):
    settle(chain)
    assert publish_until(chain.nav, lambda: twist(0.3), lambda: wheels_are(chain, 0.3 / WHEEL_RADIUS), timeout=5), \
        f"roues : {last_wheels(chain)} (attendu {0.3 / WHEEL_RADIUS} rad/s)"


def test_teleop_has_priority_over_nav2(chain):
    settle(chain)

    def both():  # Nav2 et teleop publient en meme temps
        chain.nav.publish(twist(0.3))
        return twist(0.8)
    assert publish_until(chain.teleop, both, lambda: wheels_are(chain, 0.8 / WHEEL_RADIUS), timeout=5), \
        f"roues : {last_wheels(chain)} (attendu la commande teleop {0.8 / WHEEL_RADIUS} rad/s)"


def timeline(probe, t_ref, window=1.5):
    """Chronologie autour de t_ref (diagnostic en cas d'echec)."""
    ev = [(t - t_ref, f"envoi nav vx={v}") for t, v in probe.sent if abs(t - t_ref) < window]
    ev += [(t - t_ref, f"roues {[round(x, 2) for x in w]}") for t, w in probe.wheels if abs(t - t_ref) < window]
    lines = [f"  {dt:+.3f} s  {what}" for dt, what in sorted(ev)]
    return "\n".join(lines[-40:]) + "\n--- journal control_4ws ---\n" + probe.ctrl_log()[-1500:]


def test_watchdog_stops_wheels_when_commands_stop(chain):
    settle(chain)
    assert publish_until(chain.nav, lambda: twist(0.3), lambda: wheels_are(chain, 0.3 / WHEEL_RADIUS), timeout=5)
    n_ages = len(watchdog_ages(chain.ctrl_log()))
    # plus aucune commande : les roues doivent s'arreter
    assert Probe.wait_until(lambda: wheels_are(chain, 0.0), timeout=3.0), \
        "les roues ne sont pas arretees apres l'arret des commandes\n" + chain.ctrl_log()[-1500:]
    # ... par le delai de securite, declenche selon l'age mesure par le noeud depuis sa
    # derniere reception : jamais avant cmd_timeout (0,5 s), et sans retard excessif
    assert Probe.wait_until(lambda: len(watchdog_ages(chain.ctrl_log())) > n_ages, timeout=2.0), \
        "pas de declenchement du delai de securite dans le journal de control_4ws\n" + chain.ctrl_log()[-1500:]
    age = watchdog_ages(chain.ctrl_log())[n_ages]
    assert 0.5 <= age <= 1.0, f"delai de securite declenche a {age:.2f} s (attendu entre 0,5 et 1,0 s)"


def test_e_stop_blocks_nav2_and_teleop_until_rearmed(chain):
    settle(chain)
    assert publish_until(chain.nav, lambda: twist(0.3), lambda: wheels_are(chain, 0.3 / WHEEL_RADIUS), timeout=5)

    chain.estop.publish(flag(True))
    t_estop = time.monotonic()
    wall_estop = time.time()  # comparable aux horodatages du journal de control_4ws

    def estop_logged():
        return "ARRET D'URGENCE actif" in chain.ctrl_log()

    # Nav2 continue de publier : l'arret d'urgence doit etre recu par control_4ws (journal)
    # et les roues mises a zero
    ok = publish_until(chain.nav, lambda: twist(0.3), lambda: estop_logged() and wheels_are(chain, 0.0), timeout=3)
    if not ok:
        import re
        logs = [(float(m.group(1)) - wall_estop, m.group(2)) for m in
                re.finditer(r"\[(\d+\.\d+)\] \[cmd_vel_based_4ws_control\]: (.*)", chain.ctrl_log())]
        detail = "\n".join(f"  {dt:+.3f} s  control_4ws: {txt}" for dt, txt in logs if abs(dt) < 4)
        raise AssertionError("arret d'urgence non applique dans les 3 s\n"
                             f"journal control_4ws relatif a l'envoi de /e_stop :\n{detail}\n" + timeline(chain, t_estop))

    # Nav2 et teleop insistent : rien ne doit passer
    n_before = len(chain.wheels)
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        chain.nav.publish(twist(0.3))
        chain.teleop.publish(twist(0.8))
        time.sleep(0.05)
    assert all(all(v == 0.0 for v in w) for _, w in chain.wheels[n_before:]), \
        "une commande non nulle est passee pendant l'arret d'urgence"

    # rearmement : Nav2 reprend la main
    chain.estop.publish(flag(False))
    assert publish_until(chain.nav, lambda: twist(0.3), lambda: wheels_are(chain, 0.3 / WHEEL_RADIUS), timeout=5), \
        "Nav2 n'a pas repris la main apres le rearmement"
