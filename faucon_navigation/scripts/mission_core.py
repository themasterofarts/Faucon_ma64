#!/usr/bin/env python3
"""Mission logic (core functions for mission_manager.py)"""
from __future__ import annotations

import math
import yaml
from enum import Enum
from typing import List, Tuple

_MIN_WP = 2
_MAX_WP = 100_000


class State(str, Enum):
    IDLE      = "IDLE"
    LOADING   = "LOADING"
    READY     = "READY"
    RUNNING   = "RUNNING"
    PAUSED    = "PAUSED"
    COMPLETED = "COMPLETED"
    ABORTED   = "ABORTED"
    ERROR     = "ERROR"


def validate_waypoints(yaml_str: str) -> List[dict]:
    """Parse YAML and validate waypoints. Returns list or raises ValueError."""
    try:
        data = yaml.safe_load(yaml_str)
    except Exception as e:
        raise ValueError(f"YAML invalide: {e}")

    if not isinstance(data, dict) or "waypoints" not in data:
        raise ValueError("Le YAML doit contenir une clé racine 'waypoints'")

    wps = data["waypoints"]
    if not isinstance(wps, list):
        raise ValueError("'waypoints' doit être une liste")
    if len(wps) < _MIN_WP:
        raise ValueError(f"Chemin trop court: {len(wps)} point(s), minimum {_MIN_WP}")
    if len(wps) > _MAX_WP:
        raise ValueError(f"Chemin trop long: {len(wps)} points, maximum {_MAX_WP}")

    for i, w in enumerate(wps):
        if not isinstance(w, dict):
            raise ValueError(f"Waypoint #{i}: format invalide (dict attendu)")
        for key in ("latitude", "longitude"):
            if key not in w:
                raise ValueError(f"Waypoint #{i}: clé '{key}' manquante")
        lat, lon = float(w["latitude"]), float(w["longitude"])
        if not (-90.0 <= lat <= 90.0):
            raise ValueError(f"Waypoint #{i}: latitude {lat} hors limites [-90, 90]")
        if not (-180.0 <= lon <= 180.0):
            raise ValueError(f"Waypoint #{i}: longitude {lon} hors limites [-180, 180]")

    return wps


def nearest_wp_index(positions: List[Tuple[float, float]], rx: float, ry: float) -> int:
    """Return index of the position closest to (rx, ry)."""
    best, best_d = 0, float("inf")
    for i, (px, py) in enumerate(positions):
        d = math.hypot(px - rx, py - ry)
        if d < best_d:
            best_d = d
            best = i
    return best
