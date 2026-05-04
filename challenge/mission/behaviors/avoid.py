"""Obstacle avoidance state machine.

Phased reflex: backup → turn → bypass → return-turn → settle. All durations
and motor outputs come from `MissionConfig`. Once `settle` finishes the
behavior re-enters its caller's resume state via `MissionContext.enter_state`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from .base import Behavior, MissionContext
from ..state import MissionState

if TYPE_CHECKING:
    from ..config import MissionConfig


_PHASES = ("backup", "turn", "bypass", "return", "settle")


class ObstacleAvoidance(Behavior):
    name = "avoid_obstacle"

    def __init__(self, config: "MissionConfig") -> None:
        self.config = config
        self._phase: str = ""
        self._phase_end_ts: float = 0.0
        self._resume_state: MissionState = MissionState.FOLLOW_LINE
        self._last_avoid_end_ts: float = -999.0

    @property
    def last_avoid_end_ts(self) -> float:
        return self._last_avoid_end_ts

    def reset(self) -> None:
        self._phase = ""

    def start(self, ctx: MissionContext, *, resume_state: MissionState) -> None:
        ctx.enter_state(MissionState.AVOID_OBSTACLE, "obstacle_detected")
        self._resume_state = resume_state
        self._phase = "backup"
        self._phase_end_ts = ctx.now() + self.config.avoid_backup_s

    def step(self, ctx: MissionContext) -> None:
        cfg = self.config
        now = ctx.now()

        if self._phase == "backup":
            ctx.drive(cfg.avoid_backup_speed, cfg.avoid_backup_speed)
            if now >= self._phase_end_ts:
                self._phase = "turn"
                self._phase_end_ts = now + cfg.avoid_turn_s
            return

        if self._phase == "turn":
            ctx.drive(cfg.avoid_turn_left_speed, cfg.avoid_turn_right_speed)
            if now >= self._phase_end_ts:
                self._phase = "bypass"
                self._phase_end_ts = now + cfg.avoid_bypass_s
            return

        if self._phase == "bypass":
            ctx.drive(cfg.avoid_bypass_speed, cfg.avoid_bypass_speed)
            if now >= self._phase_end_ts:
                self._phase = "return"
                self._phase_end_ts = now + cfg.avoid_return_turn_s
            return

        if self._phase == "return":
            ctx.drive(cfg.avoid_turn_right_speed, cfg.avoid_turn_left_speed)
            if now >= self._phase_end_ts:
                self._phase = "settle"
                self._phase_end_ts = now + cfg.avoid_settle_s
            return

        ctx.stop_drive()
        if now >= self._phase_end_ts:
            self._phase = ""
            self._last_avoid_end_ts = now
            ctx.enter_state(self._resume_state, "avoid_complete")


__all__ = ["ObstacleAvoidance"]
