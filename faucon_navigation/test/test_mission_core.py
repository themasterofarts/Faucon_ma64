"""Tests de mission_core (validation des waypoints, point le plus proche)."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from mission_core import validate_waypoints, nearest_wp_index  # noqa: E402


def _yaml(points):
    lines = ["waypoints:"]
    for lat, lon in points:
        lines.append(f"- {{latitude: {lat}, longitude: {lon}}}")
    return "\n".join(lines)


def test_valid_path_is_accepted():
    wps = validate_waypoints(_yaml([(43.9, 3.2), (43.9001, 3.2)]))
    assert len(wps) == 2


@pytest.mark.parametrize("text, message", [
    ("trajectory:\n- {latitude: 43.9, longitude: 3.2}\n- {latitude: 43.9, longitude: 3.2}", "waypoints"),
    (_yaml([(43.9, 3.2)]), "trop court"),
    (_yaml([(95.0, 3.2), (43.9, 3.2)]), "latitude"),
    (_yaml([(43.9, 200.0), (43.9, 3.2)]), "longitude"),
    ("waypoints:\n- {latitude: 43.9}\n- {latitude: 43.9, longitude: 3.2}", "longitude"),
    ("waypoints: [", "YAML invalide"),
])
def test_invalid_paths_are_rejected(text, message):
    with pytest.raises(ValueError, match=message):
        validate_waypoints(text)


def test_nearest_wp_index():
    positions = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    assert nearest_wp_index(positions, 2.1, 0.2) == 2
    assert nearest_wp_index(positions, -5.0, 0.0) == 0
