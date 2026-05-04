"""Mission configuration dataclass.

Defaults aim to match the vendor `Code/Server/car.py:mode_infrared` line-follow
mapping so behavior on the real robot matches what the stock firmware does
out-of-box.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple


# Softer line map for normal operation. It keeps both wheels moving forward on
# every line-visible code, which is much less twitchy on the real chassis than
# the vendor's pivot-heavy mapping.
_DEFAULT_LINE_MAP: Dict[int, Tuple[int, int]] = {
    2: (1000, 1000),    # center on line: straight forward
    4: (750, 1150),     # left sensor only: small left bias
    6: (650, 1250),     # left+center: stronger left bias
    1: (1150, 750),     # right sensor only: small right bias
    3: (1250, 650),     # right+center: stronger right bias
    0: (1000, 1000),    # all-bright: keep rolling forward slowly
}


@dataclass
class MissionConfig:
    """Single configuration object for the challenge mission."""

    # Set by main when a tuning JSON (CMA-ES / GA output) is loaded — HUD only.
    trained_params_source: str | None = None

    # Loop / cadence.
    loop_sleep_s: float = 0.05

    # Distance thresholds.
    obstacle_distance_cm: float = 18.0
    pickup_distance_cm: float = 15.0
    home_radius_m: float = 0.22

    # Line-follow behavior.
    # On real hardware, code 0 usually means all sensors on bright floor
    # (line lost), not centered on line.
    line_code_zero_is_center: bool = False
    # Freenove V2 LineSensor can report 111 on black tape depending on sensor
    # threshold/polarity. Treat it as centered by default for the real chassis;
    # operators can disable this if their board reports 111 off-line.
    line_code_seven_is_center: bool = True
    line_crawl_speed: int = 420
    line_base_speed: int = 820
    line_pd_kp: float = 145.0
    line_pd_kd: float = 60.0
    line_max_turn: int = 420
    line_turn_slowdown: float = 0.18
    line_min_forward_duty: int = 450
    line_command_map: Dict[int, Tuple[int, int]] = field(
        default_factory=lambda: dict(_DEFAULT_LINE_MAP)
    )
    # Per-tick wheel-duty change cap. Keeps a line code flip from slamming the
    # chassis sideways in one tick, which is the main source of noisy wheel
    # chatter on the real robot.
    line_max_wheel_delta: int = 700
    # When there is no line at startup, move forward briefly to put the sensor
    # bar over the tape before sweeping. Once we have seen the line, a later
    # loss first backs out of the last command, then sweeps slowly.
    line_startup_probe_s: float = 0.55
    line_startup_probe_speed: int = 500
    line_backtrack_s: float = 0.35
    line_backtrack_speed: int = 450
    line_search_turn_speed: int = 450
    line_perpendicular_recovery_s: float = 0.5

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
    # Vendor `mode_clamp_down` sweep starts servo0 from 130 and stops at 91.
    # We ramp into 130 from the carry pose (avoids a 20-deg jerk) and then
    # explicitly drive servo0 to `arm_min_angle` afterwards so the arm
    # actually reaches the bottom rather than the vendor off-by-one stop.
    drop_prep_servo0_angle: int = 130
    arm_min_angle: int = 90        # vendor servo limit floor
    arm_max_angle: int = 150       # vendor servo limit ceiling
    jaw_open_angle: int = 90       # servo1: jaws fully open
    jaw_closed_angle: int = 140    # servo1: jaws fully closed (carry/grip)
    arm_ramp_step_deg: int = 1
    arm_ramp_step_s: float = 0.012
    # After a successful pickup the ultrasonic sensor still has line-of-sight
    # to the held ball at ~5-10 cm. Suppress obstacle detection for this many
    # seconds after the pickup FSM finishes, and treat any reading closer than
    # `carry_min_obstacle_cm` as the held ball (not an obstacle).
    carry_obstacle_grace_s: float = 1.5
    carry_min_obstacle_cm: float = 12.0

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

    # Infrared robustness for real hardware:
    # Some IR boards are electrically inverted vs simulator expectations.
    # raw_code 2 (010) should usually mean center-on-line; if hardware returns
    # the opposite polarity we can either force inversion or auto-learn it.
    ir_invert_bits: bool = False
    ir_auto_invert_bits: bool = True
    # When True, the controller prints a one-line IR debug each time the
    # effective IR code changes (toggle live with the `i` key).
    ir_debug_log: bool = False

    # Manual override (single-character control scheme).
    # Each WASD press drives the wheels at the configured duty for
    # `manual_dwell_s` seconds. After that, wheels stop automatically (no
    # auto-creep). Q (or `start`) resumes autonomous behaviors. E (or `stop`)
    # halts wheels and stays in manual mode.
    manual_dwell_s: float = 0.45
    manual_speed_forward: int = 900
    manual_speed_turn: int = 1100


__all__ = ["MissionConfig"]
