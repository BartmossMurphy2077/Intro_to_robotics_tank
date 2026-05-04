"""Mission configuration dataclass.

Defaults aim to match the vendor `Code/Server/car.py:mode_infrared` line-follow
mapping so behavior on the real robot matches what the stock firmware does
out-of-box.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple


# Mirrors `Code/Server/car.py:mode_infrared` exactly (left, right) duty pairs.
_VENDOR_LINE_MAP: Dict[int, Tuple[int, int]] = {
    2: (1200, 1200),    # center on line: forward
    4: (-1500, 2500),   # left sensor only: gentle left
    6: (-2000, 4000),   # left+center: hard left
    1: (2500, -1500),   # right sensor only: gentle right
    3: (4000, -2000),   # right+center: hard right
    0: (1200, 1200),    # all-bright: treat as center (matches vendor)
}


@dataclass
class MissionConfig:
    """Single configuration object for the challenge mission."""

    # Loop / cadence.
    loop_sleep_s: float = 0.05

    # Distance thresholds.
    obstacle_distance_cm: float = 18.0
    pickup_distance_cm: float = 15.0
    home_radius_m: float = 0.22

    # Line-follow behavior.
    line_code_zero_is_center: bool = True
    line_crawl_speed: int = 260
    line_command_map: Dict[int, Tuple[int, int]] = field(
        default_factory=lambda: dict(_VENDOR_LINE_MAP)
    )

    # Obstacle avoidance motion profile.
    avoid_backup_speed: int = -1200
    avoid_backup_s: float = 0.30
    avoid_turn_left_speed: int = -1000
    avoid_turn_right_speed: int = 1000
    avoid_turn_s: float = 0.34
    avoid_bypass_speed: int = 900
    avoid_bypass_s: float = 0.42
    avoid_return_turn_s: float = 0.28
    avoid_settle_s: float = 0.12

    # Pickup (clamp) timing — delegates to vendor `mode_clamp_up/down` for the
    # actual servo+sonic alignment; these are just outer-loop timeouts.
    pre_open_before_pick: bool = True
    pick_timeout_s: float = 6.0
    drop_timeout_s: float = 4.0
    raise_arm_after_pick: bool = True
    carry_servo0_angle: int = 150
    carry_servo1_angle: int = 140
    carry_pose_settle_s: float = 0.15

    # Dead-reckoning return-to-start.
    duty_to_mps: float = 0.00022
    wheel_base_m: float = 0.16
    heading_tolerance_rad: float = 0.28
    return_home_speed: int = 950
    return_home_slow_speed: int = 700
    return_home_slow_radius_m: float = 0.55
    return_home_use_graph: bool = False

    # Vision (red-ball seek).
    use_vision: bool = False
    vision_every_n_ticks: int = 3
    seek_lock_frames: int = 3
    seek_lost_frames: int = 8
    seek_approach_radius_px: int = 30
    seek_forward_duty: int = 900
    seek_steer_kp: float = 1.0
    seek_steer_kd: float = 0.0025
    seek_commit_center_px: int = 110
    seek_commit_min_radius_px: int = 8
    seek_commit_max_distance_cm: float = 85.0
    seek_distance_agree_ratio: float = 0.65
    seek_max_s: float = 5.0
    seek_failed_cooldown_s: float = 2.0
    avoid_cooldown_s: float = 1.0
    line_recovery_cooldown_s: float = 1.0

    # Robustness.
    sonic_median_window: int = 3
    ir_majority_window: int = 3
    state_timeout_s: float = 8.0
    spiral_search_budget_s: float = 4.0

    # Manual override behavior. While the user is driving with WASD we disable
    # autonomous logic. The mission stays "latched" in manual mode until the
    # user types `auto` (or after `manual_idle_stop_s` of inactivity, when
    # wheels stop and autonomy resumes).
    manual_dwell_s: float = 0.30
    manual_idle_stop_s: float = 0.50
    manual_speed_forward: int = 900
    manual_speed_turn: int = 850

    # Vision-only mode: skip line following and obstacle avoidance entirely.
    vision_only: bool = False


__all__ = ["MissionConfig"]
