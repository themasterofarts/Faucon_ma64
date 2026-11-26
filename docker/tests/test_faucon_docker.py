"""Tests de docker/faucon_docker.sh en mode simulation (FAUCON_DOCKER_DRY_RUN=1).

En simulation, le script affiche chaque commande docker (prefixe "+ ") au lieu de
l'executer : ces tests ne demandent ni Docker ni les images.
"""
import os
import subprocess
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "faucon_docker.sh"
WARNING = "⚠  ihm_lan=true : IHM et rosbridge accessibles depuis le réseau SANS authentification."


def run(*args, **env):
    full_env = {
        "PATH": os.environ["PATH"],
        "HOME": os.environ.get("HOME", "/tmp"),
        "FAUCON_DOCKER_DRY_RUN": "1",
        "FAUCON_DOCKER_GPU": "0",
        "FAUCON_DOCKER_IMAGES_PRESENT": "1",
        "DISPLAY": ":0",
    }
    full_env.update(env)
    return subprocess.run(["bash", str(SCRIPT), *args], env=full_env,
                          capture_output=True, text=True, timeout=30)


def docker_lines(result):
    return [l for l in result.stdout.splitlines() if l.startswith("+ docker")]


def up_line(result):
    ups = [l for l in docker_lines(result) if " up " in l + " "]
    assert len(ups) == 1, result.stdout + result.stderr
    return ups[0]


def test_invalid_value_exits_1_without_docker():
    r = run("base", "use_mini=oui")
    assert r.returncode == 1
    assert "use_mini doit être true ou false" in r.stdout + r.stderr
    assert not docker_lines(r)


def test_unknown_argument_exits_1():
    r = run("base", "foo=bar")
    assert r.returncode == 1
    assert "Argument inconnu : foo=bar" in r.stdout + r.stderr
    assert not docker_lines(r)


@pytest.mark.parametrize("args", [(), ("start",)])
def test_missing_or_unknown_subcommand_exits_2(args):
    r = run(*args)
    assert r.returncode == 2
    assert "usage" in (r.stdout + r.stderr).lower()
    assert not docker_lines(r)


def test_base_defaults():
    r = run("base")
    assert r.returncode == 0, r.stderr
    out = r.stdout
    assert "ros2 launch faucon_base_desc view.launch.py" in out
    for arg in ("use_sim_time:=true", "use_mini:=false", "ihm_lan:=false", "headless:=true",
                "use_lidar_calibrator:=false", "ihm_server:=false"):
        assert arg in out
    assert "faucon_nav.launch.py" not in out
    assert "IHM_BIND=127.0.0.1" in out
    assert "compose.gpu.yaml" not in out
    assert "--build" not in up_line(r)


def test_arguments_are_passed_through():
    out = run("base", "use_mini=true", "use_lidar_calibrator=true").stdout
    assert "use_mini:=true" in out and "use_lidar_calibrator:=true" in out


def test_full_adds_navigation_after_5s():
    out = run("full").stdout
    assert "view.launch.py" in out
    assert "sleep 5" in out
    assert "ros2 launch faucon_navigation faucon_nav.launch.py use_sim_time:=true" in out


def test_ihm_lan_true_binds_all_and_warns():
    r = run("base", "ihm_lan=true")
    assert WARNING in r.stdout + r.stderr
    assert "IHM_BIND=0.0.0.0" in r.stdout
    assert "ihm_lan:=true" in r.stdout


def test_headless_false_without_gpu_has_no_gpu_file():
    r = run("base", "headless=false", FAUCON_DOCKER_GPU="0")
    assert r.returncode == 0, r.stderr
    assert "headless:=false" in r.stdout
    assert "compose.gpu.yaml" not in up_line(r)


def test_headless_false_with_gpu_adds_gpu_file():
    r = run("base", "headless=false", FAUCON_DOCKER_GPU="1")
    assert "compose.gpu.yaml" in up_line(r)


def test_builds_when_image_missing():
    r = run("base", FAUCON_DOCKER_IMAGES_PRESENT="0")
    assert "--build" in up_line(r)


@pytest.mark.parametrize("sub,expected", [("build", " build"), ("down", " down"), ("logs", " logs")])
def test_maintenance_subcommands(sub, expected):
    r = run(sub)
    assert r.returncode == 0, r.stderr
    lines = docker_lines(r)
    assert lines and all("compose" in l for l in lines)
    assert any(expected in l for l in lines)


def test_gpu_used_in_headless_mode_too():
    # les capteurs simules (camera, LiDAR) font du rendu meme sans interface
    r = run("base", FAUCON_DOCKER_GPU="1")
    assert "compose.gpu.yaml" in up_line(r)


def test_sigterm_stops_containers_promptly(tmp_path):
    """Un signal au script (kill, Ctrl+C) doit arreter les conteneurs sans attendre
    la fin de `docker compose up` (bash differe les trap tant qu'une commande au
    premier plan tourne)."""
    import signal
    import time
    calls = tmp_path / "calls.txt"
    fake = tmp_path / "docker"
    fake.write_text(
        "#!/usr/bin/env bash\n"
        f'echo "$*" >> "{calls}"\n'
        'case " $* " in *" up "*) exec sleep 30 ;; esac\n'
        "exit 0\n")
    fake.chmod(0o755)
    env = {"PATH": f"{tmp_path}:{os.environ['PATH']}", "HOME": os.environ.get("HOME", "/tmp"),
           "FAUCON_DOCKER_GPU": "0"}
    proc = subprocess.Popen(["bash", str(SCRIPT), "base"], env=env,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and not (calls.exists() and " up " in calls.read_text()):
        time.sleep(0.05)
    assert " up " in calls.read_text()
    proc.send_signal(signal.SIGTERM)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        raise AssertionError("le script n'a pas reagi au signal dans les 5 s")
    assert any(l.rstrip().endswith(" down") for l in calls.read_text().splitlines())


@pytest.mark.parametrize("headless,rviz", [("true", "false"), ("false", "true")])
def test_full_rviz_only_with_display(headless, rviz):
    # sans affichage, rviz2 s'arrete et entraine l'arret de toute la navigation
    out = run("full", f"headless={headless}").stdout
    assert f"faucon_nav.launch.py use_sim_time:=true use_rviz:={rviz}" in out


def test_headless_does_not_forward_display():
    out = run("base", DISPLAY=":1").stdout
    assert "DISPLAY=" in out and "DISPLAY=:1" not in out
