"""Configuration des tests d'integration.

Isolation : un domaine ROS dedie (par defaut derive du PID, surchargeable par
ROS_DOMAIN_ID) et une decouverte limitee a la machine, pour ne jamais interagir
avec une simulation ou un robot qui tournerait a cote.
"""
import os
import sys

import pytest

# Doit etre fait AVANT rclpy.init() (le domaine est lu a l'initialisation)
os.environ.setdefault("ROS_DOMAIN_ID", str(150 + os.getpid() % 80))
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ.setdefault("PYTHONDONTWRITEBYTECODE", "1")

sys.path.insert(0, os.path.dirname(__file__))

rclpy = pytest.importorskip("rclpy")


@pytest.fixture(scope="session", autouse=True)
def ros_context():
    from harness import ensure_rclpy
    ensure_rclpy()
    yield
    rclpy.try_shutdown()
