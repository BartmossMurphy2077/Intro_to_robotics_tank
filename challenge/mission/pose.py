"""Pose primitives and pure pose math used across the mission package."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class Pose2D:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def distance_between(a: Pose2D, b: Pose2D) -> float:
    return math.hypot(a.x_m - b.x_m, a.y_m - b.y_m)


def integrate_differential_drive(
    pose: Pose2D,
    left_mps: float,
    right_mps: float,
    wheel_base_m: float,
    dt_s: float,
) -> Pose2D:
    """Apply one differential-drive integration step. Returns a new Pose2D."""
    if dt_s <= 0.0:
        return Pose2D(pose.x_m, pose.y_m, pose.heading_rad)

    v = 0.5 * (left_mps + right_mps)
    omega = (right_mps - left_mps) / max(wheel_base_m, 1e-3)
    new_heading = normalize_angle(pose.heading_rad + omega * dt_s)
    return Pose2D(
        x_m=pose.x_m + v * math.cos(new_heading) * dt_s,
        y_m=pose.y_m + v * math.sin(new_heading) * dt_s,
        heading_rad=new_heading,
    )


__all__ = [
    "Pose2D",
    "normalize_angle",
    "distance_between",
    "integrate_differential_drive",
]
