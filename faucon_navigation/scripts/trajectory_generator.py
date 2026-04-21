#!/usr/bin/env python3
"""Independent trajectory generator module.

This module is ROS/Nav2 agnostic:
- Input: YAML containing ordered waypoints
- Output: YAML containing sampled intermediate trajectory points

Two generation modes are supported:
1) Legacy geometric corner rounding (line + tangent arc)
2) Pose-to-pose Dubins mode using input yaw values

No coordinate conversion is performed here.
"""

from __future__ import annotations

import argparse
import math
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
    min_turn_angle_deg: float = 8.0
    list_key_in: str = "waypoints"
    list_key_out: str = "trajectory"
    x_key: str = "latitude"
    y_key: str = "longitude"
    yaw_key: str = "yaw"
    include_yaw: bool = True
    use_input_yaw: bool = False


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _normalize(vx: float, vy: float) -> Optional[Point2D]:
    norm = math.hypot(vx, vy)
    if norm < 1e-12:
        return None
    return (vx / norm, vy / norm)


def _mod2pi(angle: float) -> float:
    return angle - 2.0 * math.pi * math.floor(angle / (2.0 * math.pi))


def _wrap_pi(angle: float) -> float:
    wrapped = _mod2pi(angle + math.pi) - math.pi
    if wrapped <= -math.pi:
        wrapped += 2.0 * math.pi
    return wrapped


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


def _dedupe_points(points: Sequence[Point2D], eps: float = 1e-10) -> List[Point2D]:
    out: List[Point2D] = []
    for point in points:
        if not out or _distance(out[-1], point) > eps:
            out.append(point)
    return out


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
# Legacy mode: geometric corner rounding
# ---------------------------------------------------------------------------

def _compute_rounded_corner(
    p_prev: Point2D,
    p_corner: Point2D,
    p_next: Point2D,
    desired_radius: float,
    step: float,
    min_turn_angle_rad: float,
) -> Optional[Tuple[Point2D, List[Point2D], Point2D]]:
    in_vec = (p_corner[0] - p_prev[0], p_corner[1] - p_prev[1])
    out_vec = (p_next[0] - p_corner[0], p_next[1] - p_corner[1])
    len_in = math.hypot(in_vec[0], in_vec[1])
    len_out = math.hypot(out_vec[0], out_vec[1])
    if len_in < 1e-12 or len_out < 1e-12:
        return None

    u_in = _normalize(in_vec[0], in_vec[1])
    u_out = _normalize(out_vec[0], out_vec[1])
    if u_in is None or u_out is None:
        return None

    dot_uv = _clamp(u_in[0] * u_out[0] + u_in[1] * u_out[1], -1.0, 1.0)
    turn_angle = math.acos(dot_uv)
    if turn_angle < min_turn_angle_rad or turn_angle > math.radians(170.0):
        return None

    trim = desired_radius * math.tan(turn_angle / 2.0)
    trim = min(trim, 0.45 * min(len_in, len_out))
    if trim < 1e-12:
        return None

    radius = trim / math.tan(turn_angle / 2.0)
    p_tan_in = (p_corner[0] - u_in[0] * trim, p_corner[1] - u_in[1] * trim)
    p_tan_out = (p_corner[0] + u_out[0] * trim, p_corner[1] + u_out[1] * trim)

    bisector = _normalize(-u_in[0] + u_out[0], -u_in[1] + u_out[1])
    if bisector is None:
        return None

    center_distance = radius / math.sin(turn_angle / 2.0)
    center = (p_corner[0] + bisector[0] * center_distance, p_corner[1] + bisector[1] * center_distance)

    cross_z = u_in[0] * u_out[1] - u_in[1] * u_out[0]
    if abs(cross_z) < 1e-12:
        return None

    a0 = math.atan2(p_tan_in[1] - center[1], p_tan_in[0] - center[0])
    a1 = math.atan2(p_tan_out[1] - center[1], p_tan_out[0] - center[0])
    if cross_z > 0.0:
        while a1 <= a0:
            a1 += 2.0 * math.pi
    else:
        while a1 >= a0:
            a1 -= 2.0 * math.pi

    sweep = a1 - a0
    arc_length = abs(sweep) * radius
    n_arc = max(2, int(math.ceil(arc_length / step)))

    arc_points: List[Point2D] = []
    for i in range(n_arc + 1):
        t = i / n_arc
        angle = a0 + t * sweep
        arc_points.append((center[0] + radius * math.cos(angle), center[1] + radius * math.sin(angle)))

    arc_points[0] = p_tan_in
    arc_points[-1] = p_tan_out
    return p_tan_in, arc_points, p_tan_out


def generate_trajectory_points(points: Sequence[Point2D], step: float, turn_radius: float, min_turn_angle_deg: float = 8.0) -> List[Point2D]:
    """Legacy mode: trajectory with local corner rounding (line + arc)."""
    if len(points) < 2:
        raise ValueError("At least 2 points are required")
    if step <= 0.0:
        raise ValueError("step must be > 0")
    if turn_radius <= 0.0:
        raise ValueError("turn_radius must be > 0")

    if len(points) == 2:
        return _interpolate_line(points[0], points[1], step, include_start=True)

    min_turn_angle_rad = math.radians(min_turn_angle_deg)
    path_points: List[Point2D] = []
    segment_start = points[0]

    for i in range(1, len(points) - 1):
        p_prev, p_corner, p_next = points[i - 1], points[i], points[i + 1]
        corner = _compute_rounded_corner(
            p_prev,
            p_corner,
            p_next,
            desired_radius=turn_radius,
            step=step,
            min_turn_angle_rad=min_turn_angle_rad,
        )

        if corner is None:
            path_points.extend(
                _interpolate_line(
                    segment_start,
                    p_corner,
                    step=step,
                    include_start=(len(path_points) == 0),
                )
            )
            segment_start = p_corner
            continue

        p_tan_in, arc_points, p_tan_out = corner
        path_points.extend(
            _interpolate_line(
                segment_start,
                p_tan_in,
                step=step,
                include_start=(len(path_points) == 0),
            )
        )
        path_points.extend(arc_points[1:])
        segment_start = p_tan_out

    path_points.extend(
        _interpolate_line(
            segment_start,
            points[-1],
            step=step,
            include_start=(len(path_points) == 0),
        )
    )
    return _dedupe_points(path_points)


# ---------------------------------------------------------------------------
# Pose-aware Dubins mode
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


def generate_dubins_poses(input_poses: Sequence[Pose2D], step: float, turn_radius: float) -> List[Pose2D]:
    """Generate trajectory using shortest Dubins path for each consecutive pose pair."""
    if len(input_poses) < 2:
        raise ValueError("At least 2 poses are required")
    if step <= 0.0:
        raise ValueError("step must be > 0")
    if turn_radius <= 0.0:
        raise ValueError("turn_radius must be > 0")

    out: List[Pose2D] = []
    for i in range(len(input_poses) - 1):
        start = input_poses[i]
        goal = input_poses[i + 1]
        best = _dubins_shortest_parameters(start, goal, turn_radius)
        if best is None:
            pair_poses = _fallback_pose_line(start, goal, step)
        else:
            mode, params = best
            pair_poses = _sample_dubins_pair(start, goal, mode, params, turn_radius, step)

        if not out:
            out.extend(pair_poses)
        else:
            out.extend(pair_poses[1:])

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


def read_input_poses(
    input_yaml_path: str,
    list_key: str = "waypoints",
    x_key: str = "latitude",
    y_key: str = "longitude",
    yaw_key: str = "yaw",
    require_yaw: bool = True,
) -> List[Pose2D]:
    """Read ordered poses (x, y, yaw) from YAML."""
    with open(input_yaml_path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    if not isinstance(data, dict):
        raise ValueError(f"Invalid YAML: {input_yaml_path}")
    raw_points = data.get(list_key)
    if not isinstance(raw_points, list) or len(raw_points) < 2:
        raise ValueError(f"YAML must contain at least 2 points in '{list_key}'")

    poses: List[Pose2D] = []
    for idx, item in enumerate(raw_points, start=1):
        if not isinstance(item, dict):
            raise ValueError(f"Invalid point #{idx}: dict expected")
        if x_key not in item or y_key not in item:
            raise ValueError(f"Invalid point #{idx}: missing '{x_key}'/'{y_key}'")
        if require_yaw and yaw_key not in item:
            raise ValueError(f"Invalid point #{idx}: missing yaw key '{yaw_key}'")

        x = float(item[x_key])
        y = float(item[y_key])
        if yaw_key in item:
            yaw = _wrap_pi(float(item[yaw_key]))
        else:
            yaw = 0.0
        poses.append(Pose2D(x, y, yaw))
    return poses


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


def generate_trajectory_yaml(input_yaml_path: str, output_yaml_path: str, config: TrajectoryConfig) -> Dict:
    """Complete pipeline: read -> generate -> write."""
    if config.use_input_yaw:
        input_poses = read_input_poses(
            input_yaml_path=input_yaml_path,
            list_key=config.list_key_in,
            x_key=config.x_key,
            y_key=config.y_key,
            yaw_key=config.yaw_key,
            require_yaw=True,
        )
        trajectory_poses = generate_dubins_poses(
            input_poses=input_poses,
            step=config.step,
            turn_radius=config.turn_radius,
        )
        trajectory_points: List[Point2D] = [(p.x, p.y) for p in trajectory_poses]
        trajectory_yaws = [p.yaw for p in trajectory_poses]
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

    input_points = read_input_points(
        input_yaml_path=input_yaml_path,
        list_key=config.list_key_in,
        x_key=config.x_key,
        y_key=config.y_key,
    )
    trajectory_points = generate_trajectory_points(
        points=input_points,
        step=config.step,
        turn_radius=config.turn_radius,
        min_turn_angle_deg=config.min_turn_angle_deg,
    )
    return write_trajectory_yaml(
        output_yaml_path=output_yaml_path,
        trajectory_points=trajectory_points,
        list_key=config.list_key_out,
        x_key=config.x_key,
        y_key=config.y_key,
        include_yaw=config.include_yaw,
        trajectory_yaws=None,
        yaw_key=config.yaw_key,
    )


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Independent trajectory generator")
    parser.add_argument("--input", required=True, help="Input YAML path")
    parser.add_argument("--output", required=True, help="Output YAML path")
    parser.add_argument("--step", required=True, type=float, help="Sampling step (same unit as coordinates)")
    parser.add_argument("--turn-radius", required=True, type=float, help="Turn radius (same unit as coordinates)")
    parser.add_argument("--min-turn-angle-deg", default=8.0, type=float, help="Legacy mode: minimum corner angle for arc creation")
    parser.add_argument("--input-list-key", default="waypoints")
    parser.add_argument("--output-list-key", default="trajectory")
    parser.add_argument("--x-key", default="latitude")
    parser.add_argument("--y-key", default="longitude")
    parser.add_argument("--yaw-key", default="yaw")
    parser.add_argument("--use-input-yaw", action="store_true", help="Enable pose-to-pose Dubins mode using input yaw")
    parser.add_argument("--no-yaw", action="store_true", help="Do not write yaw in output YAML")
    return parser


def main() -> None:
    parser = _build_arg_parser()
    args = parser.parse_args()

    config = TrajectoryConfig(
        step=args.step,
        turn_radius=args.turn_radius,
        min_turn_angle_deg=args.min_turn_angle_deg,
        list_key_in=args.input_list_key,
        list_key_out=args.output_list_key,
        x_key=args.x_key,
        y_key=args.y_key,
        yaw_key=args.yaw_key,
        include_yaw=not args.no_yaw,
        use_input_yaw=args.use_input_yaw,
    )
    generate_trajectory_yaml(args.input, args.output, config)
    print(f"Trajectory generated: {args.output}")


if __name__ == "__main__":
    main()
