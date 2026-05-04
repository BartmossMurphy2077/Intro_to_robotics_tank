"""Return-to-home behavior using dead-reckoned pose.

Turns toward the home anchor first (within tolerance), then drives forward
at `return_home_speed`, slowing to `return_home_slow_speed` inside the
slow-down radius. When inside `home_radius_m`, transitions to DROP_BALL.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from .base import Behavior, MissionContext
from ..pose import normalize_angle
from ..state import MissionState

if TYPE_CHECKING:
    from ..config import MissionConfig


class ReturnHome(Behavior):
    name = "return_home"

    def __init__(self, config: "MissionConfig") -> None:
        self.config = config

    def step(self, ctx: MissionContext) -> None:
        cfg = self.config
        dx = ctx.home_pose.x_m - ctx.pose.x_m
        dy = ctx.home_pose.y_m - ctx.pose.y_m
        distance = math.hypot(dx, dy)

        if distance <= cfg.home_radius_m:
            ctx.enter_state(MissionState.DROP_BALL, "home_reached")
            return

        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - ctx.pose.heading_rad)

        if abs(heading_error) > cfg.heading_tolerance_rad:
            if heading_error > 0:
                ctx.drive(-700, 700)
            else:
                ctx.drive(700, -700)
            return

        speed = cfg.return_home_speed
        if distance <= cfg.return_home_slow_radius_m:
            speed = cfg.return_home_slow_speed
        ctx.drive(speed, speed)


__all__ = ["ReturnHome"]
