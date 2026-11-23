"""Tests de la description du robot (xacro -> URDF).

Necessite xacro et le paquet faucon_base_desc trouvable (workspace source),
car ros2_control.xacro utilise $(find faucon_base_desc).
"""
import os
import xml.etree.ElementTree as ET

import pytest

xacro = pytest.importorskip("xacro")

DESC = os.path.join(os.path.dirname(__file__), "..")
LEGS = {"left_wheel_leg_joint", "right_wheel_leg_joint",
        "left_wheel_back_leg_joint", "right_wheel_back_leg_joint"}


def _urdf(path, **mappings):
    try:
        doc = xacro.process_file(path, mappings={k: str(v) for k, v in mappings.items()})
    except Exception as e:  # paquet non trouve : workspace non source
        if "faucon_base_desc" in str(e):
            pytest.skip(f"workspace non source : {e}")
        raise
    return ET.fromstring(doc.toxml())


def _leg_types(root):
    return {j.get("name"): j.get("type") for j in root.findall("joint") if j.get("name") in LEGS}


def _plugins(root):
    return {p.get("name") for p in root.iter("plugin")}


def test_default_robot_uses_ros2_control():
    root = _urdf(os.path.join(DESC, "description", "robot.urdf.xacro"))
    assert len(root.findall("ros2_control")) == 1
    assert set(_leg_types(root).values()) == {"revolute"}
    assert "gz::sim::systems::DiffDrive" not in _plugins(root)


def test_without_ros2_control_uses_diffdrive_with_fixed_legs():
    root = _urdf(os.path.join(DESC, "description", "robot.urdf.xacro"), use_ros2_control="false")
    assert not root.findall("ros2_control")
    assert set(_leg_types(root).values()) == {"fixed"}
    dd = [p for p in root.iter("plugin") if p.get("name") == "gz::sim::systems::DiffDrive"]
    assert len(dd) == 1
    assert float(dd[0].findtext("wheel_separation")) == pytest.approx(1.0)
    assert dd[0].findtext("child_frame_id") == "base_link"


def test_chassis_inertial_is_centered():
    root = _urdf(os.path.join(DESC, "description", "robot.urdf.xacro"))
    base = next(link for link in root.findall("link") if link.get("name") == "base_link")
    x = float(base.find("inertial/origin").get("xyz").split()[0])
    assert x == pytest.approx(0.0)


def test_mini_robot_generates():
    root = _urdf(os.path.join(DESC, "description_mini", "robot.urdf.xacro"))
    assert root.tag == "robot"
    assert root.findall("link")
