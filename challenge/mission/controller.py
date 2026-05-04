"""ChallengeMission — top-level state machine that composes behaviors.

This file is intentionally thin. It only:
- owns shared mission state (pose, memory, manual override flag)
- decides which behavior runs each tick
- exposes the public API (`step`, `get_status`, `reset_home_anchor`,
  `start_manual_drive`, `manual_pickup_toggle`, `set_manual_carrying_state`,
  `resume_autonomous`, `get_map_string`)

All of the actual driving logic lives in the `behaviors/` modules, each of
which can be developed and tested independently.

Manual override semantics:
    Tapping any WASD key latches the mission into MANUAL mode. While
    latched, autonomous behaviors do *not* run — there is no auto-creep.
    The wheels follow the last WASD command for `manual_dwell_s`, then
    stop on idle. The user types `auto` (or `resume`) to re-enable
    autonomous behaviors.
"""

from __future__ import annotations

import math
import time
from statistics import median
from typing import Any, Callable

from .behaviors import (
    BallPickup,
    LineFollower,
    ObstacleAvoidance,
    ReturnHome,
    VisionSeeker,
)
from .config import MissionConfig
from .memory import LineGraph, MapPoint
from .pose import Pose2D, normalize_angle
from .state import MissionState


class ChallengeMission:
    """Composes line-follow, pickup, avoidance, seek, and return-home."""

    def __init__(
        self,
        car,
        config: MissionConfig,
        *,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self.car = car
        self.config = config
        self._clock = clock

        self.state = MissionState.FOLLOW_LINE
        self._carrying_ball = False

        self.pose = Pose2D()
        self.home_pose = Pose2D()

        self._cmd_left = 0
        self._cmd_right = 0
        self._last_motion_ts = self._now()

        # Permanent memory for observed entities.
        self.obstacle_memory: list[MapPoint] = []
        self.ball_memory: list[MapPoint] = []
        self.line_memory: list[MapPoint] = []
        self.line_graph = LineGraph()
        self._return_path_nodes: list[int] = []
        self._return_path_idx = 0

        # Behaviors.
        self._line_follower = LineFollower(config, self.line_memory, self.line_graph)
        self._pickup = BallPickup(config)
        self._avoidance = ObstacleAvoidance(config)
        self._seeker = VisionSeeker(config)
        self._return_home = ReturnHome(config)

        # Manual override (latched).
        self._manual_latched: bool = False
        self._manual_until_ts: float = 0.0
        self._last_manual_cmd: tuple[int, int] = (0, 0)

        # Vision tick gate.
        self._tick_index = 0

        # Robustness state.
        self._sonic_history: list[float] = []
        self._ir_history: list[int] = []
        self._state_entry_ts: float = self._now()
        self._watchdog_pose = Pose2D()
        self._watchdog_distance_cm = -1.0
        self._watchdog_ir = 7
        self._watchdog_resets = 0
        self._state_switches = 0
        self._last_state_reason = "follow_line"
        self._ir_inverted_runtime: bool = bool(self.config.ir_invert_bits)
        self._ir_invert_votes: int = 0
        self._ir_last_raw: int = 7
        self._ir_last_used: int = 7

    # ---------- public API ----------

    def step(self) -> None:
        self._integrate_pose()
        self._tick_index += 1

        if self._manual_latched:
            self._step_manual()
            return

        distance = self._distance_cm()

        # Vision: pull a frame every N ticks.
        if (
            self.config.use_vision
            and self._tick_index % max(1, self.config.vision_every_n_ticks) == 0
        ):
            self._seeker.poll_vision(self)

        if self._apply_watchdog(distance):
            return

        if self.state == MissionState.AVOID_OBSTACLE:
            self._avoidance.step(self)
            return
        if self.state == MissionState.PICK_BALL:
            self._pickup.pick(self)
            return
        if self.state == MissionState.DROP_BALL:
            self._pickup.drop(self)
            return
        if self.state == MissionState.SEEK_BALL:
            self._seeker.step(self, distance, self._seeker.last_detection)
            return

        if self._is_obstacle(distance):
            self._remember_obstacle(distance)
            self._avoidance.start(self, resume_state=self.state)
            self._avoidance.step(self)
            return

        if self.state == MissionState.RETURN_HOME:
            self._return_home.step(self)
            return

        # FOLLOW_LINE: vision takes precedence with confident lock.
        if self.config.use_vision and not self._carrying_ball:
            ir = self._read_ir()
            line_lost = self._line_follower.is_line_lost(ir)
            if self._seeker.has_lock() and self._seeker.should_commit_to_seek(
                self,
                distance,
                ir,
                line_lost,
                self._avoidance.last_avoid_end_ts,
                self._line_follower.last_line_recovery_ts,
            ):
                self.enter_state(MissionState.SEEK_BALL, "seek_lock")
                self._seeker.step(self, distance, self._seeker.last_detection)
                return

        if self._is_pickup_distance(distance):
            self.remember_ball_here()
            self.enter_state(MissionState.PICK_BALL, "pickup_range")
            return

        self._line_follower.step(self)

    def reset_home_anchor(self) -> None:
        self.home_pose = Pose2D(self.pose.x_m, self.pose.y_m, self.pose.heading_rad)
        self.line_graph.mark_home(self.home_pose.x_m, self.home_pose.y_m)

    def is_carrying_ball(self) -> bool:
        return self._carrying_ball

    def set_manual_carrying_state(self, carrying: bool) -> None:
        self._carrying_ball = carrying
        self.enter_state(
            MissionState.RETURN_HOME if carrying else MissionState.FOLLOW_LINE,
            reason="manual_carry" if carrying else "manual_release",
        )

    def get_status(self) -> dict[str, Any]:
        return {
            "state": self.state.value,
            "state_reason": self._last_state_reason,
            "state_age_s": round(max(0.0, self._now() - self._state_entry_ts), 3),
            "x_m": self.pose.x_m,
            "y_m": self.pose.y_m,
            "heading_deg": math.degrees(self.pose.heading_rad),
            "home_m": self._distance_to_home(),
            "distance_cm": self._distance_cm(),
            "ir": self._read_ir(),
            "carrying": int(self._carrying_ball),
            "balls": len(self.ball_memory),
            "obstacles": len(self.obstacle_memory),
            "route_nodes": len(self._return_path_nodes),
            "watchdog_resets": self._watchdog_resets,
            "state_switches": self._state_switches,
            "line_lost_ticks": self._line_follower.line_lost_ticks,
            "false_seek_exits": self._seeker.false_seek_exits,
            "manual": int(self._manual_latched),
            "ir_inverted": int(self._ir_inverted_runtime),
            "ir_raw": self._ir_last_raw,
            "ir_used": self._ir_last_used,
            "line_seen": int(not self._line_follower.is_line_lost(self._ir_last_used)),
        }

    def start_manual_drive(self, key: str, duration_s: float | None = None) -> bool:
        """Latch manual mode and apply the corresponding wheel command.

        While latched, autonomous behaviors are suspended. The wheels follow
        the last WASD command for `manual_dwell_s`, then stop on idle. Type
        `auto` or call `resume_autonomous()` to re-enable autonomous logic.

        `duration_s` is accepted for backwards compatibility (callers may
        still pass a timeout) but is otherwise ignored — manual is latched.
        """
        cfg = self.config
        creep_left, creep_right = cfg.line_command_map.get(
            2, (cfg.line_crawl_speed, cfg.line_crawl_speed)
        )
        turn = cfg.manual_speed_turn

        if key == "w":
            # Use the same pair as regular center-line creep.
            left, right = int(creep_left), int(creep_right)
        elif key == "s":
            left, right = int(-creep_left), int(-creep_right)
        elif key == "a":
            left, right = -turn, turn
        elif key == "d":
            left, right = turn, -turn
        else:
            return False

        self._manual_latched = True
        self._manual_until_ts = self._now() + max(0.05, cfg.manual_dwell_s)
        self._last_manual_cmd = (left, right)
        self.drive(left, right)
        return True

    def manual_pickup_toggle(self) -> None:
        if self._carrying_ball:
            self._pickup.drop(self)
            return
        self._pickup.pick(self)

    def resume_autonomous(self) -> None:
        if self._manual_latched:
            self._manual_latched = False
            self.stop_drive()
            self.enter_state(MissionState.FOLLOW_LINE, "manual_resume")

    def is_manual_mode(self) -> bool:
        return self._manual_latched

    # ---------- MissionContext implementation (used by behaviors) ----------

    def now(self) -> float:
        return self._now()

    def sleep(self, seconds: float) -> None:
        self._sleep(seconds)

    def drive(self, left: int, right: int) -> None:
        self.car.motor.setMotorModel(int(left), int(right))
        self._cmd_left = int(left)
        self._cmd_right = int(right)

    def stop_drive(self) -> None:
        self.drive(0, 0)

    def distance_cm(self) -> float:
        return self._distance_cm()

    def read_ir(self) -> int:
        return self._read_ir()

    def set_carrying_ball(self, value: bool) -> None:
        self._carrying_ball = bool(value)

    def enter_state(self, state: MissionState, reason: str | None = None) -> None:
        if self.state == state:
            return
        self.state = state
        self._state_switches += 1
        self._state_entry_ts = self._now()
        self._last_state_reason = reason or state.value
        self._reset_watchdog_snapshot()
        if state != MissionState.SEEK_BALL:
            self._seeker.reset()
        if state == MissionState.FOLLOW_LINE:
            self._return_path_nodes = []
            self._return_path_idx = 0
            self._line_follower.reset()

    def state_age_s(self) -> float:
        return max(0.0, self._now() - self._state_entry_ts)

    def remember_obstacle(self, distance_cm: float) -> None:
        self._remember_obstacle(distance_cm)

    def remember_ball_here(self) -> None:
        self.ball_memory.append(MapPoint(self.pose.x_m, self.pose.y_m, "ball"))

    # ---------- map / debug ----------

    def get_map_string(self, size_m: float = 2.0, resolution: int = 41) -> str:
        if resolution < 3:
            resolution = 3
        half = size_m / 2.0
        step = size_m / float(resolution - 1)

        def to_idx(x_m: float, y_m: float) -> tuple[int, int]:
            rel_x = x_m - self.home_pose.x_m
            rel_y = y_m - self.home_pose.y_m
            i = int((rel_y + half) / step)
            j = int((rel_x + half) / step)
            return i, j

        grid = [[" " for _ in range(resolution)] for _ in range(resolution)]
        for p in self.line_memory:
            i, j = to_idx(p.x_m, p.y_m)
            if 0 <= i < resolution and 0 <= j < resolution:
                grid[i][j] = "-"
        for p in self.obstacle_memory:
            i, j = to_idx(p.x_m, p.y_m)
            if 0 <= i < resolution and 0 <= j < resolution:
                grid[i][j] = "X"
        for p in self.ball_memory:
            i, j = to_idx(p.x_m, p.y_m)
            if 0 <= i < resolution and 0 <= j < resolution:
                grid[i][j] = "o"

        hi, hj = to_idx(self.home_pose.x_m, self.home_pose.y_m)
        ci, cj = to_idx(self.pose.x_m, self.pose.y_m)
        if 0 <= hi < resolution and 0 <= hj < resolution:
            grid[hi][hj] = "H"
        if 0 <= ci < resolution and 0 <= cj < resolution:
            grid[ci][cj] = "*"

        rows = ["".join(row) for row in reversed(grid)]
        title = f"map(center=home size={size_m}m res={resolution})"
        return title + "\n" + "\n".join(rows)

    # ---------- internals ----------

    def _step_manual(self) -> None:
        # No autonomy. Keep last manual command active until idle timeout.
        if self._now() >= self._manual_until_ts:
            # Idle: stop wheels but stay latched in manual mode (no auto-creep).
            if self._cmd_left != 0 or self._cmd_right != 0:
                self.stop_drive()

    def _apply_watchdog(self, distance_cm: float) -> bool:
        if self.state in (MissionState.FOLLOW_LINE, MissionState.PICK_BALL, MissionState.DROP_BALL):
            self._reset_watchdog_snapshot()
            return False

        timeout_s = max(0.0, self.config.state_timeout_s)
        if timeout_s <= 0.0:
            return False
        if self._now() - self._state_entry_ts <= timeout_s:
            return False

        pose_delta = math.hypot(
            self.pose.x_m - self._watchdog_pose.x_m,
            self.pose.y_m - self._watchdog_pose.y_m,
        )
        heading_delta = abs(
            normalize_angle(self.pose.heading_rad - self._watchdog_pose.heading_rad)
        )
        current_ir = self._ir_history[-1] if self._ir_history else self._watchdog_ir
        distance_delta = (
            abs(distance_cm - self._watchdog_distance_cm)
            if distance_cm > 0 and self._watchdog_distance_cm > 0
            else 0.0
        )
        sensor_changed = current_ir != self._watchdog_ir or distance_delta > 4.0
        pose_changed = pose_delta > 0.03 or heading_delta > 0.20

        if pose_changed or sensor_changed:
            self._state_entry_ts = self._now()
            self._reset_watchdog_snapshot()
            return False

        self.stop_drive()
        self._return_path_nodes = []
        self._return_path_idx = 0
        self._avoidance.reset()
        self._seeker.reset()
        self._line_follower.reset()
        self._watchdog_resets += 1
        self.enter_state(MissionState.FOLLOW_LINE, "watchdog_reset")
        return True

    def _reset_watchdog_snapshot(self) -> None:
        self._watchdog_pose = Pose2D(
            self.pose.x_m, self.pose.y_m, self.pose.heading_rad
        )
        self._watchdog_distance_cm = self._sonic_history[-1] if self._sonic_history else -1.0
        self._watchdog_ir = self._ir_history[-1] if self._ir_history else 7

    def _is_obstacle(self, distance_cm: float) -> bool:
        if self._carrying_ball:
            return 0 < distance_cm <= self.config.obstacle_distance_cm
        if self._is_pickup_distance(distance_cm):
            return False
        return 0 < distance_cm <= self.config.obstacle_distance_cm

    def _is_pickup_distance(self, distance_cm: float) -> bool:
        return 0 < distance_cm <= self.config.pickup_distance_cm

    def _distance_cm(self) -> float:
        try:
            distance = float(self.car.sonic.get_distance())
        except Exception:
            return -1.0
        if distance > 0:
            self._sonic_history.append(distance)
            window = max(1, self.config.sonic_median_window)
            self._sonic_history = self._sonic_history[-window:]
        if not self._sonic_history:
            return -1.0
        return float(median(self._sonic_history))

    def _read_ir(self) -> int:
        try:
            raw_code = int(self.car.infrared.read_all_infrared()) & 0b111
        except Exception:
            return 7

        inverted_code = raw_code ^ 0b111
        if self.config.ir_auto_invert_bits:
            raw_useful = raw_code in (1, 2, 3, 4, 6)
            inverted_useful = inverted_code in (1, 2, 3, 4, 6)
            if inverted_useful and not raw_useful:
                self._ir_invert_votes = min(24, self._ir_invert_votes + 1)
            elif raw_useful and not inverted_useful:
                self._ir_invert_votes = max(-24, self._ir_invert_votes - 1)
            if self._ir_invert_votes >= 8:
                self._ir_inverted_runtime = True
            elif self._ir_invert_votes <= -8:
                self._ir_inverted_runtime = False

        code = inverted_code if self._ir_inverted_runtime else raw_code
        self._ir_last_raw = raw_code
        self._ir_last_used = code
        self._ir_history.append(code)
        window = max(1, self.config.ir_majority_window)
        self._ir_history = self._ir_history[-window:]
        counts: dict[int, int] = {}
        for item in self._ir_history:
            counts[item] = counts.get(item, 0) + 1
        return max(counts.items(), key=lambda item: (item[1], item[0] == code))[0]

    def _remember_obstacle(self, distance_cm: float) -> None:
        distance_m = distance_cm / 100.0
        x_m = self.pose.x_m + distance_m * math.cos(self.pose.heading_rad)
        y_m = self.pose.y_m + distance_m * math.sin(self.pose.heading_rad)
        self.obstacle_memory.append(MapPoint(x_m=x_m, y_m=y_m, kind="obstacle"))
        self.line_graph.add_obstacle(x_m, y_m)

    def _integrate_pose(self) -> None:
        now = self._now()
        dt = now - self._last_motion_ts
        self._last_motion_ts = now
        if dt <= 0.0:
            return
        dt = min(dt, 0.2)
        left_mps = self._cmd_left * self.config.duty_to_mps
        right_mps = self._cmd_right * self.config.duty_to_mps

        v = 0.5 * (left_mps + right_mps)
        omega = (right_mps - left_mps) / max(self.config.wheel_base_m, 0.001)
        self.pose.heading_rad = normalize_angle(self.pose.heading_rad + omega * dt)
        self.pose.x_m += v * math.cos(self.pose.heading_rad) * dt
        self.pose.y_m += v * math.sin(self.pose.heading_rad) * dt

    def _distance_to_home(self) -> float:
        return math.hypot(
            self.home_pose.x_m - self.pose.x_m,
            self.home_pose.y_m - self.pose.y_m,
        )

    def _now(self) -> float:
        if self._clock is not None:
            return float(self._clock())
        return time.monotonic()

    def _sleep(self, seconds: float) -> None:
        if self._clock is None:
            time.sleep(seconds)

    # Compatibility shims: tests + tuning.py reach into these names. Keeping
    # them as thin properties / mirrored fields means the old test suite
    # continues to work without reaching past the public API.
    @property
    def _line_lost_ticks(self) -> int:
        return self._line_follower.line_lost_ticks

    @property
    def _false_seek_exits(self) -> int:
        return self._seeker.false_seek_exits

    @property
    def _last_detection(self):
        return self._seeker.last_detection

    @_last_detection.setter
    def _last_detection(self, value) -> None:
        self._seeker.set_last_detection(value)

    def _follow_line_continuous(self) -> None:
        # Backwards-compat for tests that exercise this path directly.
        self._line_follower.step(self)

    def _enter_state(self, state: MissionState, reason: str | None = None) -> None:
        # Backwards-compat alias for tests that reach into the private API.
        self.enter_state(state, reason)


__all__ = ["ChallengeMission"]
