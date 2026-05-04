"""Vision-driven red-ball seek behavior.

Holds the seek-related state (lock counter, miss counter, last detection)
that the controller queries to decide when to commit to seek mode and when
to bail out. Steering uses the existing `Incremental_PID` from the vendor
client code.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from .base import Behavior, MissionContext
from ..state import MissionState

if TYPE_CHECKING:
    from ..config import MissionConfig
    from ...vision.redball import RedBallDetection


class VisionSeeker(Behavior):
    name = "seek_ball"

    def __init__(self, config: "MissionConfig") -> None:
        self.config = config
        self._pid = None
        self._lock_count: int = 0
        self._miss_count: int = 0
        self._last_detection: "Optional[RedBallDetection]" = None
        self._cooldown_until_ts: float = -999.0
        self._false_seek_exits: int = 0

    @property
    def last_detection(self):
        return self._last_detection

    @property
    def lock_count(self) -> int:
        return self._lock_count

    @property
    def miss_count(self) -> int:
        return self._miss_count

    @property
    def false_seek_exits(self) -> int:
        return self._false_seek_exits

    def set_last_detection(self, value) -> None:
        self._last_detection = value

    def reset(self) -> None:
        self._miss_count = 0
        self._lock_count = 0
        self._last_detection = None

    def reset_lock_only(self) -> None:
        self._lock_count = 0

    def has_lock(self) -> bool:
        return self._lock_count >= max(1, self.config.seek_lock_frames)

    def poll_vision(self, ctx: MissionContext) -> Optional[object]:
        if ctx.is_carrying_ball():
            self._lock_count = 0
            return None

        camera = getattr(ctx.car, "camera", None)
        if camera is None:
            self._record_miss(ctx)
            return None

        try:
            frame = camera.get_frame_bgr()
        except Exception:
            self._record_miss(ctx)
            return None

        if frame is None:
            self._record_miss(ctx)
            return None

        try:
            from ...vision.redball import detect_red_ball

            detection = detect_red_ball(frame)
        except Exception:
            self._record_miss(ctx)
            return None

        if detection is None:
            self._record_miss(ctx)
            return None

        self._lock_count += 1
        self._miss_count = 0
        self._last_detection = detection
        return detection

    def _record_miss(self, ctx: MissionContext) -> None:
        self._miss_count += 1

    def should_commit_to_seek(
        self,
        ctx: MissionContext,
        distance_cm: float,
        ir: int,
        line_lost: bool,
        last_avoid_end_ts: float,
        last_line_recovery_ts: float,
    ) -> bool:
        detection = self._last_detection
        if detection is None:
            return False
        now = ctx.now()
        if now < self._cooldown_until_ts:
            return False
        if now - last_avoid_end_ts < self.config.avoid_cooldown_s:
            return False
        if now - last_line_recovery_ts < self.config.line_recovery_cooldown_s:
            return False
        if line_lost:
            return False

        image_w, _ = detection.image_size
        center_error = abs(detection.center_xy[0] - image_w / 2.0)
        if center_error > self.config.seek_commit_center_px:
            return False
        if detection.radius_px < self.config.seek_commit_min_radius_px:
            return False
        if detection.distance_cm > self.config.seek_commit_max_distance_cm:
            return False
        if distance_cm > 0:
            agree = self.config.seek_distance_agree_ratio
            lower = detection.distance_cm * max(0.1, 1.0 - agree)
            upper = detection.distance_cm * (1.0 + agree)
            if not lower <= distance_cm <= upper:
                return False
        return True

    def step(self, ctx: MissionContext, distance_cm: float, detection=None) -> None:
        cfg = self.config
        if ctx.state_age_s() > cfg.seek_max_s:
            self._abort(ctx, "seek_timeout")
            return

        detection = detection or self._last_detection
        if detection is None:
            ctx.stop_drive()
            if self._miss_count >= max(1, cfg.seek_lost_frames):
                self._abort(ctx, "seek_lost")
            return

        is_pickup = 0 < distance_cm <= cfg.pickup_distance_cm
        if is_pickup and detection.radius_px >= cfg.seek_approach_radius_px:
            ctx.remember_ball_here()
            ctx.enter_state(MissionState.PICK_BALL, "ball_close")
            return

        if self._pid is None:
            from ...controllers.pid import Incremental_PID

            self._pid = Incremental_PID(
                P=cfg.seek_steer_kp,
                I=0.0,
                D=cfg.seek_steer_kd,
            )

        image_w, _ = detection.image_size
        self._pid.setPoint = image_w / 2.0
        steer = float(self._pid.PID_compute(detection.center_xy[0]))
        steer = max(-1100.0, min(1100.0, steer * 10.0))

        base = cfg.seek_forward_duty
        left = int(max(-4095, min(4095, base + steer)))
        right = int(max(-4095, min(4095, base - steer)))
        ctx.drive(left, right)

    def _abort(self, ctx: MissionContext, reason: str) -> None:
        self._false_seek_exits += 1
        self._lock_count = 0
        self._miss_count = 0
        if self.config.vision_only:
            # Stay in seek — stop and wait for ball to come back into view.
            ctx.stop_drive()
            ctx.enter_state(MissionState.SEEK_BALL, reason)
        else:
            self._cooldown_until_ts = ctx.now() + self.config.seek_failed_cooldown_s
            ctx.enter_state(MissionState.FOLLOW_LINE, reason)


__all__ = ["VisionSeeker"]
