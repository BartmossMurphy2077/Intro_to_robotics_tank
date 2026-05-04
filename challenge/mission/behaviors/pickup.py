"""Ball pickup behavior.

This wraps the vendor `Code/Server/car.py:Car.mode_clamp_*` lifecycle exactly
as designed by the firmware authors:

    set_mode_clamp(1) -> mode_clamp() loop until clamp_mode resets to 0
        (vendor `mode_clamp_up` aligns to ~7.5cm using the ultrasonic and
        runs the gripper servos `0` and `1`)

    set_mode_clamp(2) -> mode_clamp() loop until clamp_mode resets to 0
        (vendor `mode_clamp_down` runs the inverse servo sequence)

After a successful pick we lift the carry pose so the ball clears the
ultrasonic sensor cone (pose tunable via `MissionConfig`).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from .base import Behavior, MissionContext
from ..state import MissionState

if TYPE_CHECKING:
    from ..config import MissionConfig


_CLAMP_PICK = 1
_CLAMP_DROP = 2
_CLAMP_IDLE = 0


class BallPickup(Behavior):
    """Wraps vendor `mode_clamp_up`/`mode_clamp_down` for pick and drop."""

    name = "ball_pickup"

    def __init__(self, config: "MissionConfig") -> None:
        self.config = config

    def pick(self, ctx: MissionContext) -> None:
        ctx.stop_drive()
        if self.config.pre_open_before_pick:
            self._run_clamp_cycle(ctx, _CLAMP_DROP, self.config.drop_timeout_s)
            ctx.sleep(0.10)
        self._run_clamp_cycle(ctx, _CLAMP_PICK, self.config.pick_timeout_s)
        self._raise_carry_arm(ctx)
        ctx.set_carrying_ball(True)
        ctx.enter_state(MissionState.RETURN_HOME, "ball_picked")

    def drop(self, ctx: MissionContext) -> None:
        ctx.stop_drive()
        self._prepare_for_drop(ctx)
        self._run_clamp_cycle(ctx, _CLAMP_DROP, self.config.drop_timeout_s)
        ctx.set_carrying_ball(False)
        ctx.enter_state(MissionState.FOLLOW_LINE, "ball_dropped")

    def step(self, ctx: MissionContext) -> None:  # pragma: no cover - unused
        # Pickup is invoked imperatively (pick/drop), not via a tick loop.
        pass

    def _run_clamp_cycle(self, ctx: MissionContext, mode: int, timeout_s: float) -> None:
        car = ctx.car
        car.set_mode_clamp(mode)
        deadline = ctx.now() + max(0.0, timeout_s)
        while car.get_mode_clamp() == mode:
            car.mode_clamp()
            if ctx.now() >= deadline:
                car.set_mode_clamp(_CLAMP_IDLE)
                break

    def _raise_carry_arm(self, ctx: MissionContext) -> None:
        if not self.config.raise_arm_after_pick:
            return
        servo = getattr(ctx.car, "servo", None)
        if servo is None:
            return
        try:
            servo.setServoAngle("0", self.config.carry_servo0_angle)
            servo.setServoAngle("1", self.config.carry_servo1_angle)
            ctx.sleep(max(0.0, self.config.carry_pose_settle_s))
        except Exception:
            # Carry pose is a visibility improvement, not mission-critical.
            pass

    def _prepare_for_drop(self, ctx: MissionContext) -> None:
        """Align servo start pose with vendor `mode_clamp_down` assumptions."""
        servo = getattr(ctx.car, "servo", None)
        if servo is None:
            return
        try:
            servo.setServoAngle("0", self.config.drop_prep_servo0_angle)
            ctx.sleep(0.05)
        except Exception:
            pass


__all__ = ["BallPickup"]
