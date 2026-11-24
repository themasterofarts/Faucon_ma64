"""Tests de faucon_common.ros_utils."""
import math

import pytest

pytest.importorskip("rclpy")
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy  # noqa: E402

from faucon_common.ros_utils import (  # noqa: E402
    LATCHED_QOS, quaternion_to_yaw, yaw_to_quaternion,
)


@pytest.mark.parametrize("yaw", [0.0, 0.3, math.pi / 2, -2.5, math.pi - 1e-6])
def test_yaw_quaternion_round_trip(yaw):
    q = yaw_to_quaternion(yaw)
    assert (q.x, q.y) == (0.0, 0.0)
    assert q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2 == pytest.approx(1.0)
    assert quaternion_to_yaw(q) == pytest.approx(yaw, abs=1e-9)


def test_quaternion_to_yaw_ignores_roll_pitch():
    # lacet de 0.5 rad puis leger roulis : q = q_lacet * q_roulis (convention ZYX)
    r, y = 0.1, 0.5
    q = type("Q", (), {})()
    q.w = math.cos(y / 2) * math.cos(r / 2)
    q.x = math.cos(y / 2) * math.sin(r / 2)
    q.y = math.sin(y / 2) * math.sin(r / 2)
    q.z = math.sin(y / 2) * math.cos(r / 2)
    assert quaternion_to_yaw(q) == pytest.approx(y, abs=1e-9)


def test_latched_qos():
    assert LATCHED_QOS.depth == 1
    assert LATCHED_QOS.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert LATCHED_QOS.reliability == ReliabilityPolicy.RELIABLE
