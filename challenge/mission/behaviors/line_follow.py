"""IR line-follow behavior.

Reuses the same 3-bit IR code → (left, right) duty mapping as the vendor
`Code/Server/car.py:mode_infrared`. We do *not* delegate to that method on the
real robot because it bundles its own auto-pickup loop that fights the
mission state machine; instead the mapping lives in `MissionConfig.line_command_map`
where it can be tuned at runtime without touching vendor code.

If the line is lost (IR code 7, or 0 when configured as "lost"), we run a
short spiral search before falling back to a slow forward crawl.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from .base import Behavior, MissionContext

if TYPE_CHECKING:
    from ..config import MissionConfig
    from ..memory import LineGraph, MapPoint


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
        self._spiral_phase: int = 0  # 0 idle, 1 searching
        self._line_lost_ticks: int = 0
        self._last_line_recovery_ts: float = -999.0

    @property
    def line_lost_ticks(self) -> int:
        return self._line_lost_ticks

    @property
    def last_line_recovery_ts(self) -> float:
        return self._last_line_recovery_ts

    def reset(self) -> None:
        self._spiral_phase = 0

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
            self._search_step(ctx)
            return

        # Record line observation (cheap; debounced by the per-tick rate).
        try:
            self.line_memory.append(MapPoint(ctx.pose.x_m, ctx.pose.y_m, "line"))
            self.line_graph.add_line_point(ctx.pose.x_m, ctx.pose.y_m)
        except Exception:
            pass

        left, right = self._duty_for_ir(ir)
        ctx.drive(left, right)

    def _duty_for_ir(self, infrared_code: int) -> tuple[int, int]:
        if infrared_code == 0 and not self.config.line_code_zero_is_center:
            return self.config.line_crawl_speed, self.config.line_crawl_speed
        return self.config.line_command_map.get(
            infrared_code,
            (self.config.line_crawl_speed, self.config.line_crawl_speed),
        )

    def _search_step(self, ctx: MissionContext) -> None:
        self._line_lost_ticks += 1
        now = ctx.now()
        if self._spiral_phase == 0:
            self._spiral_entry_ts = now
            self._spiral_phase = 1

        elapsed = now - self._spiral_entry_ts
        if elapsed > self.config.spiral_search_budget_s:
            self._spiral_phase = 0
            self._last_line_recovery_ts = now
            duty = self.config.line_crawl_speed
            ctx.drive(duty, duty)
            return

        phase_s = max(0.35, self.config.spiral_search_budget_s / 4.0)
        phase = int(elapsed / phase_s) % 4
        duty = max(300, self.config.line_crawl_speed)
        if phase == 0:
            ctx.drive(-duty, duty)
        elif phase == 1:
            ctx.drive(duty, duty)
        elif phase == 2:
            ctx.drive(duty, -duty)
        else:
            ctx.drive(duty, duty)


__all__ = ["LineFollower"]
