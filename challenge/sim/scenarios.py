"""Named scenarios — build a SimWorld with a specific arena layout."""

from __future__ import annotations

import math

from .arena import Arena, BaseMarker, CircleObstacle, Polyline, RectObstacle
from .world import SimConfig, SimWorld

ROBOT_LINE_WIDTH_M = 0.12


def build_world(name: str, *, seed: int | None = None) -> SimWorld:
    builder = _SCENARIOS.get(name)
    if builder is None:
        raise ValueError(
            f"unknown scenario {name!r}; choose from {sorted(_SCENARIOS)}"
        )
    return builder(seed=seed)


def _straight_line(*, seed: int | None) -> SimWorld:
    arena = Arena(width_m=4.0, height_m=4.0)
    arena.line_polylines.append(
        Polyline(points_m=[(0.5, 2.0), (3.5, 2.0)], width_m=ROBOT_LINE_WIDTH_M)
    )
    arena.robot_start = (0.55, 2.0, 0.0)
    config = SimConfig(seed=seed)
    return SimWorld(arena=arena, config=config)


def _obstacle_detour(*, seed: int | None) -> SimWorld:
    arena = Arena(width_m=4.0, height_m=4.0)
    arena.line_polylines.append(
        Polyline(points_m=[(0.5, 2.0), (3.5, 2.0)], width_m=ROBOT_LINE_WIDTH_M)
    )
    arena.circle_obstacles.append(CircleObstacle(cx=2.0, cy=2.0, r=0.10))
    arena.robot_start = (0.55, 2.0, 0.0)
    config = SimConfig(seed=seed)
    return SimWorld(arena=arena, config=config)


def _line_with_ball(*, seed: int | None) -> SimWorld:
    arena = Arena(width_m=4.0, height_m=4.0)
    arena.line_polylines.append(
        Polyline(points_m=[(0.5, 2.0), (3.0, 2.0)], width_m=ROBOT_LINE_WIDTH_M)
    )
    arena.add_ball(cx=3.2, cy=2.0, ball_radius_m=0.04)
    arena.robot_start = (0.55, 2.0, 0.0)
    config = SimConfig(seed=seed)
    return SimWorld(arena=arena, config=config)


def _full_course(*, seed: int | None) -> SimWorld:
    arena = Arena(width_m=4.0, height_m=4.0)
    arena.line_polylines.append(
        Polyline(
            points_m=[
                (0.92, 0.66),
                (1.45, 0.42),
                (2.25, 0.48),
                (2.95, 1.05),
                (3.18, 1.75),
                (3.18, 2.42),
                (2.72, 2.88),
                (1.65, 3.05),
                (0.82, 2.58),
                (0.58, 1.70),
                (0.66, 0.98),
                (0.92, 0.66),
            ],
            width_m=ROBOT_LINE_WIDTH_M,
        )
    )
    arena.bases.extend(
        [
            BaseMarker(cx=0.68, cy=0.56, post_color_rgb=(30, 80, 215)),
            BaseMarker(cx=0.62, cy=2.72, post_color_rgb=(210, 20, 25)),
            BaseMarker(cx=3.45, cy=2.98, post_color_rgb=(50, 170, 55)),
        ]
    )
    # One obstacle near the upper leg and a ball near the far base.
    arena.circle_obstacles.append(CircleObstacle(cx=2.45, cy=2.78, r=0.10))
    arena.add_ball(cx=3.45, cy=2.98, ball_radius_m=0.04)
    # Start on the blue base just off the tape, facing the loop.
    arena.robot_start = (0.72, 0.56, 0.22)
    config = SimConfig(
        seed=seed,
        slip_sigma_per_duty=0.0008,
        heading_slip_sigma_rad=0.02,
        ultrasonic_noise_cm=0.4,
        ultrasonic_miss_prob=0.005,
        ir_flicker_prob=0.005,
    )
    return SimWorld(arena=arena, config=config)


def _noisy_sonic(*, seed: int | None) -> SimWorld:
    world = _line_with_ball(seed=seed)
    world.config.ultrasonic_noise_cm = 2.0
    world.config.ultrasonic_miss_prob = 0.10
    return world


def _flicker_ir(*, seed: int | None) -> SimWorld:
    world = _line_with_ball(seed=seed)
    world.config.ir_flicker_prob = 0.02
    return world


_SCENARIOS = {
    "straight-line": _straight_line,
    "obstacle-detour": _obstacle_detour,
    "line-with-ball": _line_with_ball,
    "full-course": _full_course,
    "noisy-sonic": _noisy_sonic,
    "flicker-ir": _flicker_ir,
}


__all__ = ["build_world"]
