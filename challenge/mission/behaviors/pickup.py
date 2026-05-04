"""Ball pickup / drop (non-blocking).

Vendor `mode_clamp_up` / `mode_clamp_down` are advanced one step per mission
tick so the runtime loop stays responsive (HUD + keyboard still work during
pickup).

Arm ramps (`servo0`) are also stepped incrementally so we never call
blocking `sleep` inside pickup."""
from __future__ import annotations

from typing import TYPE_CHECKING, Literal

from .base import Behavior, MissionContext

if TYPE_CHECKING:
    from ..config import MissionConfig

_CLAMP_PICK = 1
_CLAMP_DROP = 2
_CLAMP_IDLE = 0

StepResult = Literal["progress", "done"]


class BallPickup(Behavior):
    """Incremental pickup/drop — pump from `ChallengeMission.step` each tick."""

    name = "ball_pickup"

    def __init__(self, config: "MissionConfig") -> None:
        self.config = config
        self._last_servo0_angle: int = int(self.config.carry_servo0_angle)
        self._pick_stage: str | None = None
        self._pick_deadline: float = 0.0
        self._drop_stage: str | None = None
        self._drop_deadline: float = 0.0

    def pick(self, ctx: MissionContext) -> None:
        """Legacy synchronous path — runs the FSM to completion.

        Prefer `ensure_pick_started` + `step_pick` in the mission loop.
        """
        self.cancel(ctx.car)
        self.ensure_pick_started(ctx)
        while self._pick_stage is not None:
            if self.step_pick(ctx) == "done":
                break

    def drop(self, ctx: MissionContext) -> None:
        self.cancel(ctx.car)
        self.ensure_drop_started(ctx)
        while self._drop_stage is not None:
            if self.step_drop(ctx) == "done":
                break

    def cancel(self, car: object) -> None:
        self._pick_stage = None
        self._drop_stage = None
        self._pick_deadline = 0.0
        self._drop_deadline = 0.0
        try:
            car.set_mode_clamp(_CLAMP_IDLE)
        except Exception:
            pass

    def ensure_pick_started(self, ctx: MissionContext) -> None:
        if self._pick_stage is not None:
            return
        self._drop_stage = None
        ctx.stop_drive()
        if self.config.pre_open_before_pick:
            self._pick_stage = "jaw_open"
        else:
            self._pick_stage = "clamp_arm"
        self._pick_deadline = 0.0

    def ensure_drop_started(self, ctx: MissionContext) -> None:
        if self._drop_stage is not None:
            return
        self._pick_stage = None
        ctx.stop_drive()
        self._drop_stage = "prep_ramp"
        self._drop_deadline = 0.0

    def step_pick(self, ctx: MissionContext) -> StepResult:
        if self._pick_stage is None:
            return "done"

        cfg = self.config
        car = ctx.car

        if self._pick_stage == "jaw_open":
            if self._pick_deadline == 0.0:
                self._set_servo(ctx, "1", cfg.jaw_open_angle)
                self._pick_deadline = ctx.now() + 0.10
            elif ctx.now() >= self._pick_deadline:
                self._pick_stage = "clamp_arm"
                self._pick_deadline = 0.0
            return "progress"

        if self._pick_stage == "clamp_arm":
            if self._pick_deadline == 0.0:
                car.set_mode_clamp(_CLAMP_PICK)
                self._pick_deadline = ctx.now() + max(0.0, cfg.pick_timeout_s)
            mode = car.get_mode_clamp()
            if mode != _CLAMP_PICK:
                self._last_servo0_angle = 129
                self._pick_stage = "raise_carry"
                self._pick_deadline = 0.0
                return "progress"
            car.mode_clamp()
            if ctx.now() >= self._pick_deadline:
                car.set_mode_clamp(_CLAMP_IDLE)
                self._last_servo0_angle = 129
                self._pick_stage = "raise_carry"
                self._pick_deadline = 0.0
            return "progress"

        if self._pick_stage == "raise_carry":
            if not cfg.raise_arm_after_pick:
                self._pick_stage = "complete"
                return "progress"
            done0 = self._ramp_servo0_tick(ctx, cfg.carry_servo0_angle)
            if done0:
                servo = getattr(car, "servo", None)
                if servo is not None:
                    try:
                        servo.setServoAngle("1", cfg.carry_servo1_angle)
                    except Exception:
                        pass
                self._pick_stage = "carry_settle"
                self._pick_deadline = ctx.now() + max(0.0, cfg.carry_pose_settle_s)
            return "progress"

        if self._pick_stage == "carry_settle":
            if ctx.now() >= self._pick_deadline:
                self._pick_stage = "complete"
            return "progress"

        if self._pick_stage == "complete":
            self._pick_stage = None
            return "done"

        return "progress"

    def step_drop(self, ctx: MissionContext) -> StepResult:
        if self._drop_stage is None:
            return "done"

        cfg = self.config
        car = ctx.car

        if self._drop_stage == "prep_ramp":
            if self._ramp_servo0_tick(ctx, cfg.drop_prep_servo0_angle):
                self._drop_stage = "prep_wait"
                self._drop_deadline = ctx.now() + 0.05
            return "progress"

        if self._drop_stage == "prep_wait":
            if ctx.now() >= self._drop_deadline:
                self._drop_stage = "clamp_arm"
                self._drop_deadline = 0.0
            return "progress"

        if self._drop_stage == "clamp_arm":
            if self._drop_deadline == 0.0:
                car.set_mode_clamp(_CLAMP_DROP)
                self._drop_deadline = ctx.now() + max(0.0, cfg.drop_timeout_s)
            if car.get_mode_clamp() != _CLAMP_DROP:
                self._last_servo0_angle = 91
                self._drop_stage = "floor_ramp"
                self._drop_deadline = 0.0
                return "progress"
            car.mode_clamp()
            if ctx.now() >= self._drop_deadline:
                car.set_mode_clamp(_CLAMP_IDLE)
                self._last_servo0_angle = 91
                self._drop_stage = "floor_ramp"
                self._drop_deadline = 0.0
            return "progress"

        if self._drop_stage == "floor_ramp":
            if self._ramp_servo0_tick(ctx, cfg.arm_min_angle):
                self._drop_stage = "complete"
            return "progress"

        if self._drop_stage == "complete":
            self._drop_stage = None
            return "done"

        return "progress"

    def step(self, ctx: MissionContext) -> None:  # pragma: no cover
        pass

    def _set_servo(self, ctx: MissionContext, channel: str, angle: int) -> None:
        servo = getattr(ctx.car, "servo", None)
        if servo is None:
            return
        try:
            servo.setServoAngle(channel, angle)
        except Exception:
            pass
        if channel == "0":
            self._last_servo0_angle = int(angle)

    def _ramp_servo0_tick(self, ctx: MissionContext, target: int) -> bool:
        """Move servo0 toward target by at least one step; return True when there."""
        cfg = self.config
        servo = getattr(ctx.car, "servo", None)
        if servo is None:
            return True
        tgt = max(cfg.arm_min_angle, min(cfg.arm_max_angle, int(target)))
        cur = max(cfg.arm_min_angle, min(cfg.arm_max_angle, int(self._last_servo0_angle)))
        if cur == tgt:
            return True
        step = max(1, int(cfg.arm_ramp_step_deg))
        direction = 1 if tgt > cur else -1
        nxt = cur + direction * min(step, abs(tgt - cur))
        try:
            servo.setServoAngle("0", nxt)
        except Exception:
            pass
        self._last_servo0_angle = nxt
        return nxt == tgt


__all__ = ["BallPickup"]
