"""ChallengeMission — top-level state machine that composes behaviors.

This file is intentionally thin. It only:
- owns shared mission state (pose, memory, manual override flag)
- decides which behavior runs each tick
- exposes the public API (`step`, `get_status`, `reset_home_anchor`,
  `start_manual_drive`, `manual_pickup_toggle`, `set_manual_carrying_state`,
  `resume_autonomous`, `get_map_string`)

All of the actual driving logic lives in the `behaviors/` modules, each of
which can be developed and tested independently.

Manual vs automatic (`_operator_auto`, toggled with Q in `run_mission`):

    * **AUTO** — line follow, vision seek, ultrasonic pickup, return-home,
      and obstacle avoidance all run (mission uses tuned `MissionConfig`
      values when a JSON params file was loaded in `main.py`).

    * **MANUAL** — only WASD driving (with dwell stop), space-initiated
      pickup/drop, and while carrying a ball, return-home + avoidance still
      run so the operator can complete the challenge without re-enabling
      full AUTO.

    WASD is rejected while AUTO is on (`handle_command` + pygame runner).

Pickup / drop advance **one tick at a time** so the HUD and keyboard stay
responsive during clamp motions.

Manual wheel dwell (WASD) stops the motors after `manual_dwell_s` with no
input. **E** forces operator MANUAL + idle wheels. **Q** toggles AUTO/MANUAL.
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
        telemetry: object | None = None,
    ) -> None:
        self.car = car
        self.config = config
        self._clock = clock
        # Optional MotorTelemetry sink; None = no logging (default for tests).
        self._telemetry = telemetry
        # Last "intended" wheel command before any slew limiter clipped it. The
        # line follower stamps this before calling drive() so the telemetry can
        # show target-vs-actual divergence.
        self._last_target_l: int = 0
        self._last_target_r: int = 0

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

        # Manual wheel dwell (WASD) — only consulted when operator is in manual
        # mode (`not _operator_auto`).
        self._manual_latched: bool = False
        self._manual_until_ts: float = 0.0
        self._last_manual_cmd: tuple[int, int] = (0, 0)
        # Q toggles this in `run_mission`; default True so SimRunner / trainers
        # run autonomy without an extra setup call. Interactive `run_mission`
        # immediately calls `enter_manual_idle_at_start()` which clears this.
        self._operator_auto: bool = True

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
        self._ir_smoothed_code: int = 7
        self._ir_debug_log: bool = bool(self.config.ir_debug_log)
        self._ir_debug_last_logged: int = -1
        self._ir_debug_emit: Callable[[str], None] = print

    # ---------- public API ----------

    def step(self) -> None:
        self._integrate_pose()
        self._tick_index += 1

        if not self._operator_auto:
            distance = self._distance_cm()
            if self.state == MissionState.AVOID_OBSTACLE:
                self._avoidance.step(self)
                return
            if self.state == MissionState.PICK_BALL:
                self._pickup.ensure_pick_started(self)
                if self._pickup.step_pick(self) == "done":
                    self._carrying_ball = True
                    self.enter_state(MissionState.RETURN_HOME, "ball_picked")
                return
            if self.state == MissionState.DROP_BALL:
                self._pickup.ensure_drop_started(self)
                if self._pickup.step_drop(self) == "done":
                    self._carrying_ball = False
                    self.enter_state(MissionState.FOLLOW_LINE, "ball_dropped")
                return
            if self._carrying_ball and self.state == MissionState.RETURN_HOME:
                if self._is_obstacle(distance):
                    self._remember_obstacle(distance)
                    self._avoidance.start(self, resume_state=MissionState.RETURN_HOME)
                    self._avoidance.step(self)
                    return
                self._return_home.step(self)
                return
            self._read_ir()
            self._step_manual()
            return

        distance = self._distance_cm()

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
            self._pickup.ensure_pick_started(self)
            if self._pickup.step_pick(self) == "done":
                self._carrying_ball = True
                self.enter_state(MissionState.RETURN_HOME, "ball_picked")
            return
        if self.state == MissionState.DROP_BALL:
            self._pickup.ensure_drop_started(self)
            if self._pickup.step_drop(self) == "done":
                self._carrying_ball = False
                self.enter_state(MissionState.FOLLOW_LINE, "ball_dropped")
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
            if self._is_obstacle(distance):
                self._remember_obstacle(distance)
                self._avoidance.start(self, resume_state=MissionState.RETURN_HOME)
                self._avoidance.step(self)
                return
            self._return_home.step(self)
            return

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
            "ir": self._ir_smoothed_code,
            "carrying": int(self._carrying_ball),
            "balls": len(self.ball_memory),
            "obstacles": len(self.obstacle_memory),
            "route_nodes": len(self._return_path_nodes),
            "watchdog_resets": self._watchdog_resets,
            "state_switches": self._state_switches,
            "line_lost_ticks": self._line_follower.line_lost_ticks,
            "false_seek_exits": self._seeker.false_seek_exits,
            "manual": int(self._manual_latched),
            "operator_auto": int(self._operator_auto),
            "duty_l": self._cmd_left,
            "duty_r": self._cmd_right,
            "duty_gap": abs(self._cmd_left - self._cmd_right),
            "tuned": self.config.trained_params_source or "",
            "ir_inverted": int(self._ir_inverted_runtime),
            "ir_raw": self._ir_last_raw,
            "ir_used": self._ir_last_used,
            "line_seen": int(not self._line_follower.is_line_lost(self._ir_smoothed_code)),
        }

    def start_manual_drive(self, key: str, duration_s: float | None = None) -> bool:
        """Latch manual mode and apply the corresponding wheel command.

        Each WASD press drives at the configured duty for `manual_dwell_s`
        seconds. After dwell expires, wheels are stopped automatically
        (no auto-creep). Press Q (or call `resume_autonomous()`) to leave
        manual mode.

        `duration_s` is accepted for backwards compatibility but is
        otherwise ignored: manual mode is now latched and dwell-based.
        """
        if self._operator_auto:
            return False
        cfg = self.config
        forward = cfg.manual_speed_forward
        turn = cfg.manual_speed_turn

        if key == "w":
            left, right = forward, forward
        elif key == "s":
            left, right = -forward, -forward
        elif key == "a":
            # Rotate left in place: left tread reverses, right tread forward.
            left, right = -turn, turn
        elif key == "d":
            # Rotate right in place: left tread forward, right tread reverses.
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
            self.enter_state(MissionState.DROP_BALL, "manual_drop")
            return
        self.enter_state(MissionState.PICK_BALL, "manual_pick")

    def enter_manual_idle_at_start(self) -> None:
        """Call once at runtime startup: wheels stopped, operator manual (Q=auto)."""
        self._operator_auto = False
        self._pickup.cancel(self.car)
        self._manual_latched = True
        self._manual_until_ts = self._now()
        self._last_manual_cmd = (0, 0)
        self.stop_drive()

    def set_operator_auto(self, enabled: bool) -> None:
        """Q toggles this: full mission vs manual-drive-only (WASD when disabled)."""
        en = bool(enabled)
        self._operator_auto = en
        if not en:
            self._pickup.cancel(self.car)
            if self.state in (MissionState.PICK_BALL, MissionState.DROP_BALL):
                self.enter_state(
                    MissionState.RETURN_HOME if self._carrying_ball else MissionState.FOLLOW_LINE,
                    "operator_manual",
                )
            self._manual_latched = True
            self._manual_until_ts = self._now()
            self._last_manual_cmd = (0, 0)
            self.stop_drive()
            return
        self._manual_latched = False
        self.stop_drive()
        if self.state in (MissionState.PICK_BALL, MissionState.DROP_BALL):
            self.enter_state(
                MissionState.RETURN_HOME if self._carrying_ball else MissionState.FOLLOW_LINE,
                "operator_auto_on",
            )
            return
        next_state = MissionState.RETURN_HOME if self._carrying_ball else MissionState.FOLLOW_LINE
        self.enter_state(next_state, "operator_auto_on")

    def toggle_operator_auto(self) -> bool:
        self.set_operator_auto(not self._operator_auto)
        return self._operator_auto

    def is_operator_auto(self) -> bool:
        return self._operator_auto

    def stop_drive_latched(self) -> None:
        """E-stop style: drop to operator-manual with wheels idle."""
        self.set_operator_auto(False)

    def resume_autonomous(self) -> None:
        """Compatibility alias for runners that call `resume_autonomous`."""
        self.set_operator_auto(True)

    def is_manual_mode(self) -> bool:
        return not self._operator_auto

    def set_ir_debug(self, enabled: bool, emit: Callable[[str], None] | None = None) -> None:
        self._ir_debug_log = bool(enabled)
        if emit is not None:
            self._ir_debug_emit = emit
        # Force the next IR change to log even if it matches the previous.
        self._ir_debug_last_logged = -1

    # ---------- MissionContext implementation (used by behaviors) ----------

    def now(self) -> float:
        return self._now()

    def sleep(self, seconds: float) -> None:
        self._sleep(seconds)

    def drive(self, left: int, right: int) -> None:
        left_i, right_i = int(left), int(right)
        self.car.motor.setMotorModel(left_i, right_i)
        self._cmd_left = left_i
        self._cmd_right = right_i
        if self._telemetry is not None:
            self._telemetry.log("drive", self._telemetry_snapshot(left_i, right_i))

    def set_steer_target(self, left: int, right: int) -> None:
        """Record the line follower's intended duty before slew-limiting.

        Called by behaviors that apply their own clipping so the telemetry can
        show target-vs-actual divergence (i.e. how often the slew limiter is
        actively clipping forward acceleration).
        """
        self._last_target_l = int(left)
        self._last_target_r = int(right)

    def _telemetry_snapshot(self, drive_l: int, drive_r: int) -> dict[str, Any]:
        target_l = self._last_target_l if self._last_target_l != 0 else drive_l
        target_r = self._last_target_r if self._last_target_r != 0 else drive_r
        return {
            "ts": self._now(),
            "state": self.state.value,
            "state_age_s": max(0.0, self._now() - self._state_entry_ts),
            "reason": self._last_state_reason,
            "operator": "auto" if self._operator_auto else "manual",
            "ir_raw": self._ir_last_raw,
            "ir_used": self._ir_last_used,
            "ir_inverted": int(self._ir_inverted_runtime),
            "dist_cm": self._sonic_history[-1] if self._sonic_history else -1.0,
            "carrying": int(self._carrying_ball),
            "target_l": target_l,
            "target_r": target_r,
            "drive_l": drive_l,
            "drive_r": drive_r,
            "duty_gap": abs(drive_l - drive_r),
            "x": self.pose.x_m,
            "y": self.pose.y_m,
            "heading_deg": math.degrees(self.pose.heading_rad),
            "line_lost": self._line_follower.line_lost_ticks,
        }

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
            # Right after pickup the ultrasonic still sees the held ball at
            # 5-10cm. Suppress obstacle detection during the grace window and
            # ignore any reading closer than carry_min_obstacle_cm thereafter
            # (anything that close while carrying is the payload, not a wall).
            if self._pickup.is_carrying_grace_active(self._now()):
                return False
            if 0 < distance_cm < self.config.carry_min_obstacle_cm:
                return False
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
            self._ir_smoothed_code = 7
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
        smoothed = max(counts.items(), key=lambda item: (item[1], item[0] == code))[0]
        if self._line_follower.is_line_lost(smoothed) and not self._line_follower.is_line_lost(code):
            smoothed = code
        self._ir_smoothed_code = int(smoothed)

        if self._ir_debug_log and smoothed != self._ir_debug_last_logged:
            self._ir_debug_last_logged = smoothed
            line_seen = "NO" if self._line_follower.is_line_lost(smoothed) else "yes"
            self._ir_debug_emit(
                f"[ir] raw={raw_code:03b} used={smoothed:03b} "
                f"inv={'1' if self._ir_inverted_runtime else '0'} line={line_seen}"
            )
        return smoothed

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
