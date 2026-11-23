"""Tests du generateur de trajectoires (les deux copies).

Verifie que le yaw ecrit est en convention ENU (0 = est, pi/2 = nord) et que
la sortie est acceptee par mission_manager (cle "waypoints").
"""
import importlib.util
import math
import os
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
COPIES = {
    "navigation": os.path.join(REPO, "faucon_navigation", "scripts", "trajectory_generator.py"),
    "gui": os.path.join(REPO, "faucon_path_planning", "gui", "trajectory_generator.py"),
}

sys.path.insert(0, os.path.join(REPO, "faucon_navigation", "scripts"))
from mission_core import validate_waypoints  # noqa: E402


def _load(name, path):
    spec = importlib.util.spec_from_file_location(f"tg_{name}", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # requis par les dataclasses
    spec.loader.exec_module(module)
    return module


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@pytest.fixture
def two_rows(tmp_path):
    # rang 1 vers le nord, demi-tour, rang 2 vers le sud
    src = tmp_path / "in.yaml"
    src.write_text(
        "waypoints:\n"
        "- {latitude: 43.89997, longitude: 3.19997}\n"
        "- {latitude: 43.90007, longitude: 3.19997}\n"
        "- {latitude: 43.90007, longitude: 3.20003}\n"
        "- {latitude: 43.89997, longitude: 3.20003}\n"
    )
    return src


@pytest.mark.parametrize("copy", sorted(COPIES))
def test_output_is_accepted_and_yaw_is_enu(copy, two_rows, tmp_path):
    tg = _load(copy, COPIES[copy])
    out = tmp_path / f"out_{copy}.yaml"
    tg.generate_trajectory_yaml(str(two_rows), str(out), tg.TrajectoryConfig(step=0.5, turn_radius=2.0))

    text = out.read_text()
    wps = validate_waypoints(text)  # leve ValueError si la cle n'est pas "waypoints"
    assert len(wps) > 10

    data = yaml.safe_load(text)["waypoints"]
    assert data[0]["yaw"] == pytest.approx(math.pi / 2, abs=1e-6)    # depart vers le nord
    assert data[-1]["yaw"] == pytest.approx(-math.pi / 2, abs=1e-6)  # arrivee vers le sud

    # le yaw doit suivre la direction de deplacement reelle (ENU)
    errors = []
    for a, b in zip(data, data[1:]):
        dn = math.radians(b["latitude"] - a["latitude"]) * 6378137.0
        de = math.radians(b["longitude"] - a["longitude"]) * 6378137.0 * math.cos(math.radians(a["latitude"]))
        if math.hypot(dn, de) < 1e-3:
            continue
        errors.append(abs(_wrap(a["yaw"] - math.atan2(dn, de))))
    # tolerance : corde vs tangente dans les virages (pas 0.5 m, rayon 2 m)
    assert max(errors) < math.radians(10)


def test_both_copies_produce_the_same_trajectory(two_rows, tmp_path):
    outputs = []
    for name in sorted(COPIES):
        tg = _load(name, COPIES[name])
        out = tmp_path / f"same_{name}.yaml"
        tg.generate_trajectory_yaml(str(two_rows), str(out), tg.TrajectoryConfig(step=0.5, turn_radius=2.0))
        outputs.append(out.read_text())
    assert outputs[0] == outputs[1], "les deux copies du generateur ont diverge"
