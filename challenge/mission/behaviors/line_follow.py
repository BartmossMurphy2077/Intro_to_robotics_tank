"""IR line-follow behavior.

Reuses the same 3-bit IR code → (left, right) duty mapping as the vendor
`Code/Server/car.py:mode_infrared`. We do *not* delegate to that method on the
real robot because it bundles its own auto-pickup loop that fights the
mission state machine; instead the mapping lives in `MissionConfig.line_command_map`
where it can be tuned at runtime without touching vendor code.

The raw vendor map issues ±4000 duty for hard turns, which on a single ~50 ms
tick is enough to rotate the chassis 60–90° — the chassis ends up perpendicular
to the line. We therefore (a) ship a softer command map in `default.json`, and
(b) clip the per-tick wheel-duty delta to `line_max_wheel_delta`, so even a
noisy IR flip cannot slam the chassis sideways in one tick.

If the line is lost, startup probes forward briefly to put the sensor bar over
the tape. After the line was seen once, loss recovery first backs out of the
last good command, then runs a bounded sweep and slow forward crawl.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from .base import Behavior, MissionContext

if TYPE_CHECKING:
    from ..config import MissionConfig
    from ..memory import LineGraph, MapPoint


# IR codes that mean "line is to the LEFT of center" — used to pick a recovery
# rotation direction after the spiral search times out.
_LINE_LEFT_CODES = frozenset({4, 6})
_LINE_RIGHT_CODES = frozenset({1, 3})
_LINE_ERROR = {
    4: -1.0,
    6: -2.0,
    2: 0.0,
    7: 0.0,
    1: 1.0,
    3: 2.0,
}


class LineFollower(Behavior):
    name = "line_follow"

    def __init__(
        self,
        config: "MissionConfig",
        line_memory: "list[MapPoint]",
        line_graph: "LineGraph",
    ) -> None:
        self.config = config
        self.line_memory = line_memory
        self.line_graph = line_graph
        self._spiral_entry_ts: float = 0.0
        self._spiral_phase: int = 0  # 0 idle, 1 searching, 2 perpendicular recovery
        self._line_lost_ticks: int = 0
        self._last_line_recovery_ts: float = -999.0

        # Slew limiter state: last commanded wheel duties (zero on first tick).
        self._prev_left: int = 0
        self._prev_right: int = 0
        # Direction the line was last observed before it went missing
        # (-1 = left of center, +1 = right of center, 0 = unknown).
        self._last_line_dir: int = 0
        # Re-acquire debounce: tick count of consecutive line-visible reads.
        self._reacquire_ticks: int = 0
        self._line_seen_ever: bool = False
        self._lost_entry_ts: float = 0.0
        self._last_good_left: int = 0
        self._last_good_right: int = 0
        self._last_error: float = 0.0

    @property
    def line_lost_ticks(self) -> int:
        return self._line_lost_ticks

    @property
    def last_line_recovery_ts(self) -> float:
        return self._last_line_recovery_ts

    def reset(self) -> None:
        self._spiral_phase = 0
        self._prev_left = 0
        self._prev_right = 0
        self._reacquire_ticks = 0
        self._lost_entry_ts = 0.0
        self._last_error = 0.0

    def is_line_lost(self, infrared_code: int) -> bool:
        if infrared_code == 7 and self.config.line_code_seven_is_center:
            return False
        if infrared_code == 7:
            return True
        if infrared_code == 0 and not self.config.line_code_zero_is_center:
            return True
        return False

    def step(self, ctx: MissionContext) -> None:
        from ..memory import MapPoint  # avoid circular at import time

        ir = ctx.read_ir()
        if self.is_line_lost(ir):
            self._reacquire_ticks = 0
            self._search_step(ctx)
            return

        if self._spiral_phase != 0:
            # The controller already smooths IR reads. Once that smoothed code
            # is visible, leave search immediately so we do not drive past the
            # line at low speed while waiting for a second confirmatory tick.
            self._spiral_phase = 0
            self._last_line_recovery_ts = ctx.now()

        self._reacquire_ticks = 0
        self._line_lost_ticks = 0
        self._line_seen_ever = True
        self._update_last_line_dir(ir)

        # Record line observation (cheap; debounced by the per-tick rate).
        try:
            self.line_memory.append(MapPoint(ctx.pose.x_m, ctx.pose.y_m, "line"))
            self.line_graph.add_line_point(ctx.pose.x_m, ctx.pose.y_m)
        except Exception:
            pass

        target_l, target_r = self._pd_duty_for_ir(ir)
        left, right = self._apply_steer_limit(target_l, target_r)
        if hasattr(ctx, "set_steer_target"):
            ctx.set_steer_target(target_l, target_r)
        ctx.drive(left, right)
        self._prev_left, self._prev_right = left, right
        self._last_good_left, self._last_good_right = left, right

    def _duty_for_ir(self, infrared_code: int) -> tuple[int, int]:
        if infrared_code == 7 and self.config.line_code_seven_is_center:
            return self.config.line_command_map.get(7, self.config.line_command_map.get(2, (1000, 1000)))
        if infrared_code == 0 and not self.config.line_code_zero_is_center:
            return self.config.line_crawl_speed, self.config.line_crawl_speed
        return self.config.line_command_map.get(
            infrared_code,
            (self.config.line_crawl_speed, self.config.line_crawl_speed),
        )

    def _apply_steer_limit(self, left: int, right: int) -> tuple[int, int]:
        cap = max(0, int(self.config.line_max_wheel_delta))
        if cap <= 0:
            return left, right
        return (
            _clip_delta(self._prev_left, left, cap),
            _clip_delta(self._prev_right, right, cap),
        )

    def _update_last_line_dir(self, infrared_code: int) -> None:
        if infrared_code in _LINE_LEFT_CODES:
            self._last_line_dir = -1
        elif infrared_code in _LINE_RIGHT_CODES:
            self._last_line_dir = +1
        # Code 2 (centered) leaves the previous direction in place — useful if
        # we lose the line again after a brief centering moment.

    def _pd_duty_for_ir(self, infrared_code: int) -> tuple[int, int]:
        error = _line_error_for_ir(infrared_code, self.config.line_code_seven_is_center)
        derivative = error - self._last_error
        self._last_error = error

        raw_turn = self.config.line_pd_kp * error + self.config.line_pd_kd * derivative
        max_turn = max(0, int(self.config.line_max_turn))
        turn = max(-max_turn, min(max_turn, int(round(raw_turn))))

        slowdown = max(0.0, min(0.8, float(self.config.line_turn_slowdown)))
        base = int(round(self.config.line_base_speed * (1.0 - slowdown * min(1.0, abs(error) / 2.0))))
        base = max(self.config.line_crawl_speed, base)

        left = base + turn
        right = base - turn
        return max(0, left), max(0, right)

    def _search_step(self, ctx: MissionContext) -> None:
        self._line_lost_ticks += 1
        now = ctx.now()
        if self._spiral_phase == 0:
            self._spiral_entry_ts = now
            self._lost_entry_ts = now
            self._spiral_phase = 1

        lost_elapsed = now - self._lost_entry_ts
        if not self._line_seen_ever and lost_elapsed < max(0.0, self.config.line_startup_probe_s):
            self._drive_target(
                ctx,
                self.config.line_startup_probe_speed,
                self.config.line_startup_probe_speed,
            )
            return

        if (
            self._line_seen_ever
            and lost_elapsed < max(0.0, self.config.line_backtrack_s)
            and (self._last_good_left != 0 or self._last_good_right != 0)
        ):
            left = -_clamp_abs(self._last_good_left, self.config.line_backtrack_speed)
            right = -_clamp_abs(self._last_good_right, self.config.line_backtrack_speed)
            self._drive_target(ctx, left, right)
            return

        elapsed = max(0.0, now - self._spiral_entry_ts - max(0.0, self.config.line_backtrack_s))
        budget = max(0.5, self.config.spiral_search_budget_s)
        recovery_budget = max(0.0, self.config.line_perpendicular_recovery_s)
        total_budget = budget + recovery_budget

        if elapsed > total_budget:
            # Both spiral and perpendicular recovery exhausted — give up and
            # creep forward, leaving _spiral_phase set so the debounce on
            # re-acquire still applies.
            self._last_line_recovery_ts = now
            duty = self.config.line_crawl_speed
            self._drive_target(ctx, duty, duty)
            return

        if elapsed > budget:
            # Perpendicular-recovery phase: rotate slowly toward the last seen
            # line direction. If we never saw a direction, default to a left
            # rotation (matches the spiral's first phase).
            self._spiral_phase = 2
            duty = max(250, self.config.line_search_turn_speed)
            direction = self._last_line_dir if self._last_line_dir != 0 else -1
            if direction < 0:
                target_l, target_r = -duty, duty
            else:
                target_l, target_r = duty, -duty
            self._drive_target(ctx, target_l, target_r)
            return

        # Spiral phase: cycle through rotate-left, forward, rotate-right, forward.
        phase_s = max(0.35, budget / 4.0)
        phase = int(elapsed / phase_s) % 4
        turn = max(250, self.config.line_search_turn_speed)
        crawl = max(250, self.config.line_crawl_speed)
        direction = self._last_line_dir if self._last_line_dir != 0 else -1
        if phase == 0:
            target_l, target_r = (-turn, turn) if direction < 0 else (turn, -turn)
        elif phase == 1:
            target_l, target_r = crawl, crawl
        elif phase == 2:
            target_l, target_r = (turn, -turn) if direction < 0 else (-turn, turn)
        else:
            target_l, target_r = crawl, crawl
        self._drive_target(ctx, target_l, target_r)

    def _drive_target(self, ctx: MissionContext, target_l: int, target_r: int) -> None:
        left, right = self._apply_steer_limit(target_l, target_r)
        if hasattr(ctx, "set_steer_target"):
            ctx.set_steer_target(target_l, target_r)
        ctx.drive(left, right)
        self._prev_left, self._prev_right = left, right


def _clip_delta(prev: int, target: int, cap: int) -> int:
    """Move from `prev` toward `target` by at most `cap` units."""
    delta = target - prev
    if delta > cap:
        return prev + cap
    if delta < -cap:
        return prev - cap
    return target


def _clamp_abs(value: int, cap: int) -> int:
    cap = max(0, int(cap))
    if cap <= 0:
        return int(value)
    return max(-cap, min(cap, int(value)))


def _line_error_for_ir(infrared_code: int, seven_is_center: bool = True) -> float:
    if infrared_code == 7 and not seven_is_center:
        return 0.0
    return _LINE_ERROR.get(infrared_code, 0.0)


__all__ = ["LineFollower"]
