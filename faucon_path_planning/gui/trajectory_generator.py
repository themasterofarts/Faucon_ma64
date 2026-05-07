#!/usr/bin/env python3
"""Independent trajectory generator module.

This module is ROS/Nav2 agnostic:
- Input: YAML containing ordered waypoints
- Output: YAML containing sampled intermediate trajectory points

Single generation mode:
- rows as straight lines + Dubins turns only between rows

Pipeline:
- GNSS lat/lon -> local metric frame
- trajectory generation in meters
- local metric frame -> GNSS lat/lon
"""

from __future__ import annotations

import argparse
import math
import os
import warnings
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

Point2D = Tuple[float, float]


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class TrajectoryConfig:
    step: float
    turn_radius: float
    list_key_in: str = "waypoints"
    list_key_out: str = "trajectory"
    x_key: str = "latitude"
    y_key: str = "longitude"
    yaw_key: str = "yaw"
    include_yaw: bool = True
    gnss_to_local: bool = True
    origin_lat: Optional[float] = None
    origin_lon: Optional[float] = None
    earth_radius_m: float = 6378137.0


def _distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _mod2pi(angle: float) -> float:
    return angle - 2.0 * math.pi * math.floor(angle / (2.0 * math.pi))


def _wrap_pi(angle: float) -> float:
    wrapped = _mod2pi(angle + math.pi) - math.pi
    if wrapped <= -math.pi:
        wrapped += 2.0 * math.pi
    return wrapped


def _resolve_origin(
    points_latlon: Sequence[Point2D],
    origin_lat: Optional[float],
    origin_lon: Optional[float],
) -> Point2D:
    if origin_lat is None and origin_lon is None:
        return points_latlon[0]
    if origin_lat is None or origin_lon is None:
        raise ValueError("Both origin_lat and origin_lon must be provided together")
    return (float(origin_lat), float(origin_lon))


def _latlon_to_local_xy(
    latitude: float,
    longitude: float,
    origin_lat: float,
    origin_lon: float,
    earth_radius_m: float,
) -> Point2D:
    # Local tangent plane approximation:
    # x axis = north (latitude direction), y axis = east (longitude direction).
    lat0_rad = math.radians(origin_lat)
    dlat = math.radians(latitude - origin_lat)
    dlon = math.radians(longitude - origin_lon)
    x_north = earth_radius_m * dlat
    y_east = earth_radius_m * math.cos(lat0_rad) * dlon
    return (x_north, y_east)


def _local_xy_to_latlon(
    x_local: float,
    y_local: float,
    origin_lat: float,
    origin_lon: float,
    earth_radius_m: float,
) -> Point2D:
    lat0_rad = math.radians(origin_lat)
    cos_lat0 = math.cos(lat0_rad)
    if abs(cos_lat0) < 1e-12:
        raise ValueError("Invalid origin latitude for conversion (cos(latitude) too close to zero)")

    latitude = origin_lat + math.degrees(x_local / earth_radius_m)
    longitude = origin_lon + math.degrees(y_local / (earth_radius_m * cos_lat0))
    return (latitude, longitude)


def _points_latlon_to_local(
    points_latlon: Sequence[Point2D],
    origin_lat: float,
    origin_lon: float,
    earth_radius_m: float,
) -> List[Point2D]:
    return [
        _latlon_to_local_xy(lat, lon, origin_lat, origin_lon, earth_radius_m)
        for lat, lon in points_latlon
    ]


def _points_local_to_latlon(
    points_local: Sequence[Point2D],
    origin_lat: float,
    origin_lon: float,
    earth_radius_m: float,
) -> List[Point2D]:
    return [
        _local_xy_to_latlon(x, y, origin_lat, origin_lon, earth_radius_m)
        for x, y in points_local
    ]


def _poses_latlon_to_local(
    poses_latlon: Sequence[Pose2D],
    origin_lat: float,
    origin_lon: float,
    earth_radius_m: float,
) -> List[Pose2D]:
    out: List[Pose2D] = []
    for pose in poses_latlon:
        x_local, y_local = _latlon_to_local_xy(
            pose.x,
            pose.y,
            origin_lat,
            origin_lon,
            earth_radius_m,
        )
        out.append(Pose2D(x_local, y_local, pose.yaw))
    return out


def _poses_local_to_latlon(
    poses_local: Sequence[Pose2D],
    origin_lat: float,
    origin_lon: float,
    earth_radius_m: float,
) -> List[Pose2D]:
    out: List[Pose2D] = []
    for pose in poses_local:
        lat, lon = _local_xy_to_latlon(
            pose.x,
            pose.y,
            origin_lat,
            origin_lon,
            earth_radius_m,
        )
        out.append(Pose2D(lat, lon, pose.yaw))
    return out


def _interpolate_line(start: Point2D, goal: Point2D, step: float, include_start: bool) -> List[Point2D]:
    dist = _distance(start, goal)
    if dist < 1e-12:
        return [start] if include_start else []

    n = max(1, int(math.ceil(dist / step)))
    pts: List[Point2D] = []
    for i in range(n + 1):
        if i == 0 and not include_start:
            continue
        t = i / n
        pts.append((start[0] + t * (goal[0] - start[0]), start[1] + t * (goal[1] - start[1])))
    return pts


def _dedupe_poses(poses: Sequence[Pose2D], eps: float = 1e-10) -> List[Pose2D]:
    out: List[Pose2D] = []
    for pose in poses:
        if not out:
            out.append(pose)
            continue
        last = out[-1]
        if _distance((last.x, last.y), (pose.x, pose.y)) > eps or abs(_wrap_pi(pose.yaw - last.yaw)) > 1e-6:
            out.append(pose)
    return out


# ---------------------------------------------------------------------------
# Dubins primitives
# ---------------------------------------------------------------------------

def _dubins_lsl(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    tmp0 = d + math.sin(alpha) - math.sin(beta)
    p2 = 2.0 + d * d - 2.0 * math.cos(alpha - beta) + 2.0 * d * (math.sin(alpha) - math.sin(beta))
    if p2 < 0.0:
        return None
    tmp1 = math.atan2(math.cos(beta) - math.cos(alpha), tmp0)
    t = _mod2pi(-alpha + tmp1)
    p = math.sqrt(p2)
    q = _mod2pi(beta - tmp1)
    return (t, p, q)


def _dubins_rsr(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    tmp0 = d - math.sin(alpha) + math.sin(beta)
    p2 = 2.0 + d * d - 2.0 * math.cos(alpha - beta) + 2.0 * d * (math.sin(beta) - math.sin(alpha))
    if p2 < 0.0:
        return None
    tmp1 = math.atan2(math.cos(alpha) - math.cos(beta), tmp0)
    t = _mod2pi(alpha - tmp1)
    p = math.sqrt(p2)
    q = _mod2pi(-beta + tmp1)
    return (t, p, q)


def _dubins_lsr(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    p2 = -2.0 + d * d + 2.0 * math.cos(alpha - beta) + 2.0 * d * (math.sin(alpha) + math.sin(beta))
    if p2 < 0.0:
        return None
    p = math.sqrt(p2)
    tmp2 = math.atan2(-math.cos(alpha) - math.cos(beta), d + math.sin(alpha) + math.sin(beta)) - math.atan2(-2.0, p)
    t = _mod2pi(-alpha + tmp2)
    q = _mod2pi(-beta + tmp2)
    return (t, p, q)


def _dubins_rsl(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    p2 = -2.0 + d * d + 2.0 * math.cos(alpha - beta) - 2.0 * d * (math.sin(alpha) + math.sin(beta))
    if p2 < 0.0:
        return None
    p = math.sqrt(p2)
    tmp2 = math.atan2(math.cos(alpha) + math.cos(beta), d - math.sin(alpha) - math.sin(beta)) - math.atan2(2.0, p)
    t = _mod2pi(alpha - tmp2)
    q = _mod2pi(beta - tmp2)
    return (t, p, q)


def _dubins_rlr(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    tmp0 = (6.0 - d * d + 2.0 * math.cos(alpha - beta) + 2.0 * d * (math.sin(alpha) - math.sin(beta))) / 8.0
    if abs(tmp0) > 1.0:
        return None
    p = _mod2pi(2.0 * math.pi - math.acos(tmp0))
    t = _mod2pi(alpha - math.atan2(math.cos(alpha) - math.cos(beta), d - math.sin(alpha) + math.sin(beta)) + p / 2.0)
    q = _mod2pi(alpha - beta - t + p)
    return (t, p, q)


def _dubins_lrl(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    tmp0 = (6.0 - d * d + 2.0 * math.cos(alpha - beta) + 2.0 * d * (-math.sin(alpha) + math.sin(beta))) / 8.0
    if abs(tmp0) > 1.0:
        return None
    p = _mod2pi(2.0 * math.pi - math.acos(tmp0))
    t = _mod2pi(-alpha - math.atan2(math.cos(alpha) - math.cos(beta), d + math.sin(alpha) - math.sin(beta)) + p / 2.0)
    q = _mod2pi(_mod2pi(beta) - alpha - t + p)
    return (t, p, q)


def _dubins_shortest_parameters(start: Pose2D, goal: Pose2D, radius: float) -> Optional[Tuple[str, Tuple[float, float, float]]]:
    if radius <= 0.0:
        raise ValueError("radius must be > 0")

    dx = goal.x - start.x
    dy = goal.y - start.y
    d_euclid = math.hypot(dx, dy)
    d = d_euclid / radius
    theta = math.atan2(dy, dx)
    alpha = _mod2pi(start.yaw - theta)
    beta = _mod2pi(goal.yaw - theta)

    candidates: List[Tuple[str, Tuple[float, float, float], float]] = []
    planners = [
        ("LSL", _dubins_lsl),
        ("RSR", _dubins_rsr),
        ("LSR", _dubins_lsr),
        ("RSL", _dubins_rsl),
        ("RLR", _dubins_rlr),
        ("LRL", _dubins_lrl),
    ]
    for mode, fn in planners:
        params = fn(alpha, beta, d)
        if params is None:
            continue
        total = params[0] + params[1] + params[2]
        candidates.append((mode, params, total))

    if not candidates:
        return None

    best = min(candidates, key=lambda c: c[2])
    return best[0], best[1]


def _integrate_segment(pose: Pose2D, mode: str, ds: float, radius: float) -> Pose2D:
    x, y, yaw = pose.x, pose.y, pose.yaw

    if mode == "S":
        return Pose2D(x + ds * math.cos(yaw), y + ds * math.sin(yaw), yaw)

    dtheta = ds / radius
    if mode == "L":
        yaw2 = yaw + dtheta
        x2 = x + radius * (math.sin(yaw2) - math.sin(yaw))
        y2 = y - radius * (math.cos(yaw2) - math.cos(yaw))
        return Pose2D(x2, y2, _wrap_pi(yaw2))

    if mode == "R":
        yaw2 = yaw - dtheta
        x2 = x + radius * (math.sin(yaw) - math.sin(yaw2))
        y2 = y + radius * (math.cos(yaw2) - math.cos(yaw))
        return Pose2D(x2, y2, _wrap_pi(yaw2))

    raise ValueError(f"Unknown mode: {mode}")


def _sample_dubins_pair(start: Pose2D, goal: Pose2D, mode: str, params: Tuple[float, float, float], radius: float, step: float) -> List[Pose2D]:
    seg_lengths = [params[0] * radius, params[1] * radius, params[2] * radius]
    poses: List[Pose2D] = [Pose2D(start.x, start.y, _wrap_pi(start.yaw))]
    current = poses[0]

    for seg_mode, seg_len in zip(mode, seg_lengths):
        if seg_len <= 1e-12:
            continue
        n = max(1, int(math.ceil(seg_len / step)))
        ds = seg_len / n
        for _ in range(n):
            current = _integrate_segment(current, seg_mode, ds, radius)
            poses.append(current)

    poses[-1] = Pose2D(goal.x, goal.y, _wrap_pi(goal.yaw))
    return poses


def _fallback_pose_line(start: Pose2D, goal: Pose2D, step: float) -> List[Pose2D]:
    pts = _interpolate_line((start.x, start.y), (goal.x, goal.y), step=step, include_start=True)
    if len(pts) == 1:
        pts = [pts[0], pts[0]]
    yaws = _compute_yaws(pts)
    yaws[0] = _wrap_pi(start.yaw)
    yaws[-1] = _wrap_pi(goal.yaw)
    return [Pose2D(x, y, yaws[i]) for i, (x, y) in enumerate(pts)]


def _heading(start: Point2D, goal: Point2D) -> float:
    return math.atan2(goal[1] - start[1], goal[0] - start[0])


def _line_segment_poses(start: Point2D, goal: Point2D, step: float, include_start: bool) -> List[Pose2D]:
    pts = _interpolate_line(start, goal, step=step, include_start=include_start)
    if not pts:
        return []
    yaw = _wrap_pi(_heading(start, goal))
    return [Pose2D(x, y, yaw) for x, y in pts]


def generate_turns_only_dubins_poses(points: Sequence[Point2D], step: float, turn_radius: float) -> List[Pose2D]:
    """Rows are straight lines; only inter-row connectors are Dubins turns.

    Expected input order: [row1_in, row1_out, row2_in, row2_out, ...].
    """
    if len(points) < 2:
        raise ValueError("At least 2 points are required")
    if len(points) % 2 != 0:
        raise ValueError("This row-based mode requires an even number of points (entry/exit per row)")
    if step <= 0.0:
        raise ValueError("step must be > 0")
    if turn_radius <= 0.0:
        raise ValueError("turn_radius must be > 0")

    if len(points) == 2:
        return _line_segment_poses(points[0], points[1], step=step, include_start=True)

    rows = [(points[i], points[i + 1]) for i in range(0, len(points), 2)]
    out: List[Pose2D] = []

    for row_idx, (row_start, row_end) in enumerate(rows):
        line_poses = _line_segment_poses(
            row_start,
            row_end,
            step=step,
            include_start=(len(out) == 0),
        )
        if line_poses:
            out.extend(line_poses)

        if row_idx == len(rows) - 1:
            continue

        next_start, next_end = rows[row_idx + 1]
        yaw_start = _wrap_pi(_heading(row_start, row_end))
        yaw_goal = _wrap_pi(_heading(next_start, next_end))
        turn_start = Pose2D(row_end[0], row_end[1], yaw_start)
        turn_goal = Pose2D(next_start[0], next_start[1], yaw_goal)

        best = _dubins_shortest_parameters(turn_start, turn_goal, turn_radius)
        if best is None:
            turn_poses = _fallback_pose_line(turn_start, turn_goal, step)
        else:
            mode, params = best
            turn_poses = _sample_dubins_pair(turn_start, turn_goal, mode, params, turn_radius, step)

        if not out:
            out.extend(turn_poses)
        else:
            out.extend(turn_poses[1:])

    return _dedupe_poses(out)


# ---------------------------------------------------------------------------
# YAML IO helpers
# ---------------------------------------------------------------------------

def read_input_points(
    input_yaml_path: str,
    list_key: str = "waypoints",
    x_key: str = "latitude",
    y_key: str = "longitude",
) -> List[Point2D]:
    """Read ordered 2D points from YAML."""
    with open(input_yaml_path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    if not isinstance(data, dict):
        raise ValueError(f"Invalid YAML: {input_yaml_path}")
    raw_points = data.get(list_key)
    if not isinstance(raw_points, list) or len(raw_points) < 2:
        raise ValueError(f"YAML must contain at least 2 points in '{list_key}'")

    points: List[Point2D] = []
    for idx, item in enumerate(raw_points, start=1):
        if not isinstance(item, dict):
            raise ValueError(f"Invalid point #{idx}: dict expected")
        if x_key not in item or y_key not in item:
            raise ValueError(f"Invalid point #{idx}: missing '{x_key}'/'{y_key}'")
        points.append((float(item[x_key]), float(item[y_key])))
    return points


def _compute_yaws(points: Sequence[Point2D]) -> List[float]:
    if len(points) < 2:
        return [0.0] * len(points)

    yaws: List[float] = []
    for i, (x, y) in enumerate(points):
        if i < len(points) - 1:
            nx, ny = points[i + 1]
            yaws.append(math.atan2(ny - y, nx - x))
        else:
            px, py = points[i - 1]
            yaws.append(math.atan2(y - py, x - px))
    return yaws


def write_trajectory_yaml(
    output_yaml_path: str,
    trajectory_points: Sequence[Point2D],
    list_key: str = "trajectory",
    x_key: str = "latitude",
    y_key: str = "longitude",
    include_yaw: bool = True,
    trajectory_yaws: Optional[Sequence[float]] = None,
    yaw_key: str = "yaw",
) -> Dict:
    """Write trajectory points (and optionally yaw) to YAML."""
    if len(trajectory_points) == 0:
        raise ValueError("No trajectory points to write")

    output_list: List[Dict] = []
    if include_yaw:
        if trajectory_yaws is None:
            yaws = _compute_yaws(trajectory_points)
        else:
            if len(trajectory_yaws) != len(trajectory_points):
                raise ValueError("trajectory_yaws length must match trajectory_points")
            yaws = list(trajectory_yaws)
    else:
        yaws = []

    for i, (x, y) in enumerate(trajectory_points):
        item: Dict[str, float] = {x_key: float(x), y_key: float(y)}
        if include_yaw:
            item[yaw_key] = float(_wrap_pi(yaws[i]))
        output_list.append(item)

    out_data: Dict[str, List[Dict]] = {list_key: output_list}
    with open(output_yaml_path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(out_data, stream, sort_keys=False, default_flow_style=False)
    return out_data


def _build_plot_path(output_yaml_path: str) -> str:
    root, _ = os.path.splitext(output_yaml_path)
    return f"{root}.png"


def visualize_trajectory(
    input_points: Sequence[Point2D],
    trajectory_points: Sequence[Point2D],
    output_png_path: str,
    x_key: str,
    y_key: str,
    title: str,
) -> str:
    """Create a PNG visualization of input points and generated trajectory."""
    os.environ.setdefault("MPLCONFIGDIR", os.path.join("/tmp", "matplotlib"))
    os.makedirs(os.environ["MPLCONFIGDIR"], exist_ok=True)
    try:
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="Unable to import Axes3D.*", category=UserWarning)
            import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            "matplotlib is required for --visualize. Install with: pip install matplotlib"
        ) from exc

    # Human-friendly map-like view for lat/lon inputs.
    map_like = x_key.lower() == "latitude" and y_key.lower() == "longitude"
    if map_like:
        in_x = [p[1] for p in input_points]      # longitude on x-axis
        in_y = [p[0] for p in input_points]      # latitude on y-axis
        tr_x = [p[1] for p in trajectory_points]
        tr_y = [p[0] for p in trajectory_points]
        x_label, y_label = "longitude", "latitude"
    else:
        in_x = [p[0] for p in input_points]
        in_y = [p[1] for p in input_points]
        tr_x = [p[0] for p in trajectory_points]
        tr_y = [p[1] for p in trajectory_points]
        x_label, y_label = x_key, y_key

    plt.figure(figsize=(10, 8))
    plt.plot(tr_x, tr_y, "-", linewidth=2.0, color="#d62728", label=f"trajectory ({len(trajectory_points)} pts)")
    plt.plot(in_x, in_y, "o", color="black", markersize=6, label=f"input ({len(input_points)} pts)")

    plt.scatter([in_x[0]], [in_y[0]], color="green", s=70, label="start", zorder=5)
    plt.scatter([in_x[-1]], [in_y[-1]], color="purple", s=70, label="end", zorder=5)

    for idx, (xv, yv) in enumerate(zip(in_x, in_y), start=1):
        plt.annotate(str(idx), (xv, yv), textcoords="offset points", xytext=(4, 4), fontsize=8)

    plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.axis("equal")
    plt.tight_layout()
    plt.savefig(output_png_path, dpi=220)
    plt.close()
    return output_png_path


def generate_trajectory_yaml(input_yaml_path: str, output_yaml_path: str, config: TrajectoryConfig) -> Dict:
    """Complete pipeline: read -> generate -> write."""
    input_points_raw = read_input_points(
        input_yaml_path=input_yaml_path,
        list_key=config.list_key_in,
        x_key=config.x_key,
        y_key=config.y_key,
    )

    if config.gnss_to_local:
        origin_lat, origin_lon = _resolve_origin(
            input_points_raw,
            config.origin_lat,
            config.origin_lon,
        )
        input_points_local = _points_latlon_to_local(
            points_latlon=input_points_raw,
            origin_lat=origin_lat,
            origin_lon=origin_lon,
            earth_radius_m=config.earth_radius_m,
        )
    else:
        input_points_local = list(input_points_raw)

    trajectory_poses_local = generate_turns_only_dubins_poses(
        points=input_points_local,
        step=config.step,
        turn_radius=config.turn_radius,
    )

    if config.gnss_to_local:
        trajectory_poses_out = _poses_local_to_latlon(
            poses_local=trajectory_poses_local,
            origin_lat=origin_lat,
            origin_lon=origin_lon,
            earth_radius_m=config.earth_radius_m,
        )
    else:
        trajectory_poses_out = trajectory_poses_local

    trajectory_points: List[Point2D] = [(p.x, p.y) for p in trajectory_poses_out]
    trajectory_yaws = [p.yaw for p in trajectory_poses_out]

    return write_trajectory_yaml(
        output_yaml_path=output_yaml_path,
        trajectory_points=trajectory_points,
        list_key=config.list_key_out,
        x_key=config.x_key,
        y_key=config.y_key,
        include_yaw=config.include_yaw,
        trajectory_yaws=trajectory_yaws if config.include_yaw else None,
        yaw_key=config.yaw_key,
    )


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Independent trajectory generator")
    parser.add_argument("--input", required=True, help="Input YAML path")
    parser.add_argument("--output", required=True, help="Output YAML path")
    parser.add_argument("--step", required=True, type=float, help="Sampling step in meters (default GNSS->local pipeline)")
    parser.add_argument("--turn-radius", required=True, type=float, help="Turn radius in meters (default GNSS->local pipeline)")
    parser.add_argument("--input-list-key", default="waypoints")
    parser.add_argument("--output-list-key", default="trajectory")
    parser.add_argument("--x-key", default="latitude")
    parser.add_argument("--y-key", default="longitude")
    parser.add_argument("--yaw-key", default="yaw")
    parser.add_argument(
        "--no-gnss-to-local",
        action="store_true",
        help="Disable GNSS->local->GNSS conversion (only if your input is already in local metric coordinates)",
    )
    parser.add_argument("--origin-lat", type=float, default=None, help="Optional origin latitude for GNSS->local conversion")
    parser.add_argument("--origin-lon", type=float, default=None, help="Optional origin longitude for GNSS->local conversion")
    parser.add_argument("--earth-radius-m", type=float, default=6378137.0, help="Earth radius used by GNSS<->local conversion")
    parser.add_argument("--no-yaw", action="store_true", help="Do not write yaw in output YAML")
    parser.add_argument("--visualize", action="store_true", help="Generate a PNG preview plot")
    parser.add_argument("--plot-output", default=None, help="PNG output path (default: same as --output with .png)")
    return parser


def main() -> None:
    parser = _build_arg_parser()
    args = parser.parse_args()

    config = TrajectoryConfig(
        step=args.step,
        turn_radius=args.turn_radius,
        list_key_in=args.input_list_key,
        list_key_out=args.output_list_key,
        x_key=args.x_key,
        y_key=args.y_key,
        yaw_key=args.yaw_key,
        include_yaw=not args.no_yaw,
        gnss_to_local=not args.no_gnss_to_local,
        origin_lat=args.origin_lat,
        origin_lon=args.origin_lon,
        earth_radius_m=args.earth_radius_m,
    )
    out_data = generate_trajectory_yaml(args.input, args.output, config)
    print(f"Trajectory generated: {args.output}")

    if args.visualize:
        input_points = read_input_points(
            input_yaml_path=args.input,
            list_key=config.list_key_in,
            x_key=config.x_key,
            y_key=config.y_key,
        )
        raw_traj = out_data[config.list_key_out]
        trajectory_points = [(float(p[config.x_key]), float(p[config.y_key])) for p in raw_traj]
        plot_path = args.plot_output if args.plot_output else _build_plot_path(args.output)
        visualize_trajectory(
            input_points=input_points,
            trajectory_points=trajectory_points,
            output_png_path=plot_path,
            x_key=config.x_key,
            y_key=config.y_key,
            title="Dubins turns-only trajectory",
        )
        print(f"Plot generated: {plot_path}")


if __name__ == "__main__":
    main()
