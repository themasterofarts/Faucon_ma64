"""Outils communs aux tests d'integration (vrais noeuds ROS 2, sans Gazebo).

- NodeProcess : lance un noeud (script Python du depot ou executable ROS) dans un
  sous-processus, et l'arrete proprement (SIGINT, comme Ctrl+C). Si la variable
  FAUCON_COVERAGE=1 est definie, les scripts Python sont lances sous coverage.py
  (mode parallele) pour mesurer la couverture des noeuds eux-memes.
- Probe : noeud rclpy du test (publications, abonnements, services factices) qui
  tourne dans un thread, avec wait_until() pour attendre une condition.
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
import time
from typing import Callable, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
COVERAGE_RC = os.path.join(REPO, ".coveragerc")


def repo_path(*parts: str) -> str:
    return os.path.join(REPO, *parts)


def ros_executable(package: str, executable: str) -> str:
    """Chemin d'un executable ROS installe (ex. twist_mux)."""
    from ament_index_python.packages import get_package_prefix
    return os.path.join(get_package_prefix(package), "lib", package, executable)


class NodeProcess:
    """Noeud ROS lance en sous-processus, arrete par SIGINT."""

    def __init__(self, cmd: list[str], name: str, python_script: bool = False):
        env = dict(os.environ)
        # faucon_common depuis les sources (fonctionne aussi sans colcon build)
        env["PYTHONPATH"] = repo_path("faucon_common") + os.pathsep + env.get("PYTHONPATH", "")
        # journaux ROS ecrits immediatement : les tests les lisent pendant l'execution
        env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "0"
        env["PYTHONUNBUFFERED"] = "1"
        if python_script:
            prefix = [sys.executable]
            if os.environ.get("FAUCON_COVERAGE") == "1":
                prefix += ["-m", "coverage", "run", "--parallel-mode", f"--rcfile={COVERAGE_RC}"]
            cmd = prefix + cmd
        self.name = name
        self.log_path = os.path.join(os.environ.get("FAUCON_TEST_LOGS", "/tmp"), f"it_{name}_{os.getpid()}.log")
        self._log = open(self.log_path, "w")
        self.proc = subprocess.Popen(
            cmd, env=env, stdout=self._log, stderr=subprocess.STDOUT, start_new_session=True,
            # SIGINT remis par defaut (il peut etre ignore quand le test tourne en arriere-plan)
            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL),
        )

    @classmethod
    def python(cls, script: str, name: str, args: Optional[list[str]] = None) -> "NodeProcess":
        return cls([script] + (args or []), name, python_script=True)

    def alive(self) -> bool:
        return self.proc.poll() is None

    def stop(self, timeout: float = 10.0) -> int:
        if self.alive():
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()
        self._log.close()
        return self.proc.returncode

    def log(self) -> str:
        if not self._log.closed:
            self._log.flush()
        with open(self.log_path) as f:
            return f.read()


class Probe(Node):
    """Noeud du test : tourne dans un thread dedie (MultiThreadedExecutor)."""

    def __init__(self, name: str = "it_probe"):
        super().__init__(name)
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._executor.shutdown(timeout_sec=2.0)
        self.destroy_node()

    @staticmethod
    def wait_until(predicate: Callable[[], bool], timeout: float, period: float = 0.02) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(period)
        return predicate()


def publish_for(pub, msg_factory: Callable[[], object], duration: float, rate_hz: float) -> None:
    """Publie a frequence fixe pendant `duration` secondes (comme Nav2 ou l'IHM)."""
    end = time.monotonic() + duration
    while time.monotonic() < end:
        pub.publish(msg_factory())
        time.sleep(1.0 / rate_hz)


def publish_until(pub, msg_factory: Callable[[], object], predicate: Callable[[], bool],
                  timeout: float, rate_hz: float = 20.0) -> bool:
    """Publie a frequence fixe jusqu'a ce que `predicate` soit vrai (ou timeout).

    A preferer a publish_for() + assertion immediate : le transport DDS peut retarder
    ponctuellement la reception de plusieurs centaines de ms (observe en CI).
    """
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        pub.publish(msg_factory())
        if predicate():
            return True
        time.sleep(1.0 / rate_hz)
    return predicate()


def watchdog_ages(log_text: str) -> list[float]:
    """Ages mesures par le noeud lui-meme lors des declenchements de son delai de
    securite ("Aucune commande /cmd_vel depuis X s"), dans l'ordre du journal."""
    import re
    return [float(x) for x in re.findall(r"Aucune commande /cmd_vel depuis ([0-9.]+) s", log_text)]


def ensure_rclpy() -> None:
    if not rclpy.ok():
        rclpy.init()
