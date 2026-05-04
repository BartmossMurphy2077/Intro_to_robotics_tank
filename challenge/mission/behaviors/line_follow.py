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

If the line is lost (IR code 7, or 0 when configured as "lost"), we run a
short spiral search; if that times out we rotate-in-place toward the last
seen line direction before falling back to a slow forward crawl.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from .base import Behavior, MissionContext

if TYPE_CHECKING:
    from ..config import MissionConfig
    from ..memory import LineGraph, MapPoint


# IR codes that mean "line visible somewhere under the array".
_LINE_VISIBLE_CODES = frozenset({1, 2, 3, 4, 6})
# IR codes that mean "line is to the LEFT of center" — used to pick a recovery
# rotation direction after the spiral search times out.
_LINE_LEFT_CODES = frozenset({4, 6})
_LINE_RIGHT_CODES = frozenset({1, 3})


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

    def is_line_lost(self, infrared_code: int) -> bool:
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

        # Debounce: wait for two consecutive line-visible reads before exiting
        # a search/recovery sequence. A single noisy frame should not snap us
        # back into bang-bang oscillation.
        if self._spiral_phase != 0:
            self._reacquire_ticks += 1
            if self._reacquire_ticks < 2:
                # Hold the search motion one more tick.
                self._search_step(ctx)
                return
            # Two clean ticks — exit search, fall through to normal follow.
            self._spiral_phase = 0
            self._last_line_recovery_ts = ctx.now()

        self._reacquire_ticks = 0
        self._line_lost_ticks = 0
        self._update_last_line_dir(ir)

        # Record line observation (cheap; debounced by the per-tick rate).
        try:
            self.line_memory.append(MapPoint(ctx.pose.x_m, ctx.pose.y_m, "line"))
            self.line_graph.add_line_point(ctx.pose.x_m, ctx.pose.y_m)
        except Exception:
            pass

        target_l, target_r = self._duty_for_ir(ir)
        left, right = self._apply_steer_limit(target_l, target_r)
        if hasattr(ctx, "set_steer_target"):
            ctx.set_steer_target(target_l, target_r)
        ctx.drive(left, right)
        self._prev_left, self._prev_right = left, right

    def _duty_for_ir(self, infrared_code: int) -> tuple[int, int]:
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

    def _search_step(self, ctx: MissionContext) -> None:
        self._line_lost_ticks += 1
        now = ctx.now()
        if self._spiral_phase == 0:
            self._spiral_entry_ts = now
            self._spiral_phase = 1

        elapsed = now - self._spiral_entry_ts
        budget = max(0.5, self.config.spiral_search_budget_s)
        recovery_budget = max(0.0, self.config.line_perpendicular_recovery_s)
        total_budget = budget + recovery_budget

        if elapsed > total_budget:
            # Both spiral and perpendicular recovery exhausted — give up and
            # creep forward, leaving _spiral_phase set so the debounce on
            # re-acquire still applies.
            self._last_line_recovery_ts = now
            duty = self.config.line_crawl_speed
            left, right = self._apply_steer_limit(duty, duty)
            if hasattr(ctx, "set_steer_target"):
                ctx.set_steer_target(duty, duty)
            ctx.drive(left, right)
            self._prev_left, self._prev_right = left, right
            return

        if elapsed > budget:
            # Perpendicular-recovery phase: rotate slowly toward the last seen
            # line direction. If we never saw a direction, default to a left
            # rotation (matches the spiral's first phase).
            self._spiral_phase = 2
            duty = max(300, self.config.line_crawl_speed)
            direction = self._last_line_dir if self._last_line_dir != 0 else -1
            if direction < 0:
                target_l, target_r = -duty, duty
            else:
                target_l, target_r = duty, -duty
            left, right = self._apply_steer_limit(target_l, target_r)
            if hasattr(ctx, "set_steer_target"):
                ctx.set_steer_target(target_l, target_r)
            ctx.drive(left, right)
            self._prev_left, self._prev_right = left, right
            return

        # Spiral phase: cycle through rotate-left, forward, rotate-right, forward.
        phase_s = max(0.35, budget / 4.0)
        phase = int(elapsed / phase_s) % 4
        duty = max(300, self.config.line_crawl_speed)
        if phase == 0:
            target_l, target_r = -duty, duty
        elif phase == 1:
            target_l, target_r = duty, duty
        elif phase == 2:
            target_l, target_r = duty, -duty
        else:
            target_l, target_r = duty, duty
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


__all__ = ["LineFollower"]
