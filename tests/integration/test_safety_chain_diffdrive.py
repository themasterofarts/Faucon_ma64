"""Integration : securite en mode DiffDrive (twist_mux + diffdrive_safety reels).

En mode DiffDrive (robot mini, use_ros2_control:=false), /cmd_vel va directement
au plugin Gazebo, qui garde la derniere vitesse : diffdrive_safety doit forcer
une vitesse nulle sur /e_stop et quand les commandes s'arretent.
"""
import time

import pytest
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from harness import NodeProcess, Probe, publish_for, publish_until, repo_path, ros_executable, watchdog_ages


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
    # Les compteurs du graphe ne garantissent pas que la liaison twist_mux -> diffdrive_safety
    # sur /cmd_vel est etablie (decouverte DDS plus lente en CI). Echauffement de bout en
    # bout : on envoie des commandes jusqu'a voir diffdrive_safety forcer un zero (delai
    # de securite), ce qui prouve qu'il recoit bien /cmd_vel.
    probe.safety_log = safety.log
    warm = False
    end = time.monotonic() + 30
    while not warm and time.monotonic() < end:
        publish_for(probe.nav, lambda: twist(0.3), 0.3, 20)
        warm = Probe.wait_until(lambda: probe.cmd and probe.cmd[-1][1] == 0.0, timeout=1.5)
    assert warm, "diffdrive_safety ne reagit pas a /cmd_vel :\n" + safety.log()[-2000:]
    time.sleep(0.3)
    yield probe
    probe.estop.publish(flag(False))
    probe.close()
    assert safety.stop() == 0, "diffdrive_safety ne s'arrete pas proprement :\n" + safety.log()[-2000:]
    mux.stop()


def held(probe):
    """Vitesse que garderait le DiffDrive Gazebo : derniere valeur recue."""
    return probe.cmd[-1][1] if probe.cmd else None


# NB sur les temps : en CI, un abonne DDS (ici diffdrive_safety) peut ponctuellement ne
# rien recevoir pendant plus de 0,5 s alors que la sonde, abonnee au meme topic, recoit
# tout (observe dans les journaux de la CI). Le delai de securite est donc verifie
# d'apres l'age mesure par le noeud depuis sa derniere reception (son journal).

def test_watchdog_forces_zero_when_commands_stop(chain):
    assert publish_until(chain.nav, lambda: twist(0.3), lambda: held(chain) == pytest.approx(0.3), timeout=5)
    publish_for(chain.nav, lambda: twist(0.3), 0.3, 20)
    n_ages = len(watchdog_ages(chain.safety_log()))
    # plus aucune commande : diffdrive_safety doit forcer une vitesse nulle
    assert Probe.wait_until(lambda: held(chain) == 0.0, timeout=3.0), \
        "vitesse non forcee a zero apres l'arret des commandes :\n" + chain.safety_log()[-1500:]
    # ... par son delai de securite, declenche selon l'age mesure depuis sa derniere
    # reception : jamais avant cmd_timeout (0,5 s), et sans retard excessif
    assert Probe.wait_until(lambda: len(watchdog_ages(chain.safety_log())) > n_ages, timeout=2.0), \
        "pas de declenchement du delai de securite dans le journal :\n" + chain.safety_log()[-1500:]
    age = watchdog_ages(chain.safety_log())[n_ages]
    assert 0.5 <= age <= 1.0, f"delai de securite declenche a {age:.2f} s (attendu entre 0,5 et 1,0 s)"


def test_e_stop_forces_zero_while_nav2_insists(chain):
    time.sleep(0.8)
    assert publish_until(chain.nav, lambda: twist(0.3), lambda: held(chain) == pytest.approx(0.3), timeout=5)
    chain.estop.publish(flag(True))

    def estop_logged():
        return "ARRET D'URGENCE actif" in chain.safety_log()

    # Nav2 continue de publier : l'arret d'urgence doit etre recu et la vitesse forcee a zero
    assert publish_until(chain.nav, lambda: twist(0.3), lambda: estop_logged() and held(chain) == 0.0, timeout=3), \
        "arret d'urgence non applique dans les 3 s :\n" + chain.safety_log()[-1500:]
    # twist_mux peut recevoir /e_stop un peu apres diffdrive_safety (transport) : on laisse
    # 0,5 s au verrou pour s'appliquer, puis plus aucune commande Nav2 ne doit passer
    publish_for(chain.nav, lambda: twist(0.3), 0.5, 20)
    t_check = time.monotonic()
    publish_for(chain.nav, lambda: twist(0.3), 1.0, 20)  # Nav2 insiste
    after = [v for t, v in chain.cmd if t > t_check]
    assert after and all(v == 0.0 for v in after), "une commande non nulle est passee pendant l'arret d'urgence"

    chain.estop.publish(flag(False))
    assert publish_until(chain.nav, lambda: twist(0.3), lambda: held(chain) == pytest.approx(0.3), timeout=5), \
        "Nav2 n'a pas repris la main apres le rearmement"
