"""Tests for the editable JSON config loader and the new line/carry knobs."""

from __future__ import annotations

import json
from pathlib import Path

from challenge.mission import MissionConfig
from challenge.mission.behaviors.line_follow import LineFollower, _clip_delta
from challenge.mission.behaviors.pickup import BallPickup
from challenge.mission.config_loader import apply_config_dict, load_config_file


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = REPO_ROOT / "challenge" / "config" / "default.json"


def test_default_config_loads_and_applies() -> None:
    cfg = MissionConfig()
    data = load_config_file(DEFAULT_CONFIG)
    skipped = apply_config_dict(cfg, data)

    # Comment keys must be stripped silently.
    for key in skipped:
        assert key.startswith("_") or key not in {f.name for f in cfg.__dataclass_fields__.values()}

    # Editable file ships a softer line map than the vendor defaults; both
    # wheels stay positive on every line-visible code so the chassis always
    # advances rather than pivoting in place mid-line.
    for code in (1, 2, 3, 4, 6):
        left, right = cfg.line_command_map[code]
        assert left >= 0 and right >= 0, f"code {code}: ({left}, {right}) has reverse component"
        assert left + right >= 1200, f"code {code}: not enough forward bias"
    assert cfg.line_code_seven_is_center is True
    assert cfg.line_command_map[7] == (700, 700)
    # New tunables are present.
    assert cfg.line_base_speed > cfg.line_crawl_speed
    assert cfg.line_pd_kp > 0
    assert cfg.line_pd_kd >= 0
    assert cfg.line_max_turn > 0
    assert cfg.line_min_forward_duty >= cfg.line_crawl_speed
    assert 300 <= cfg.line_max_wheel_delta <= 650
    assert cfg.line_startup_probe_speed >= cfg.line_crawl_speed
    assert cfg.line_backtrack_s > 0
    assert cfg.carry_obstacle_grace_s == 1.5
    assert cfg.carry_min_obstacle_cm == 12.0


def test_apply_config_dict_ignores_unknown_keys() -> None:
    cfg = MissionConfig()
    skipped = apply_config_dict(cfg, {"obstacle_distance_cm": 9.5, "not_a_real_field": 42})
    assert cfg.obstacle_distance_cm == 9.5
    assert "not_a_real_field" in skipped


def test_clip_delta_caps_swings() -> None:
    # +cap when target far above prev
    assert _clip_delta(0, 4000, 1500) == 1500
    # -cap when target far below prev
    assert _clip_delta(1500, -4000, 1500) == 0
    # passthrough when within cap
    assert _clip_delta(500, 800, 1500) == 800


def test_line_follower_rate_limit_seeds_from_zero() -> None:
    # First tick from a stopped chassis cannot exceed the cap on either wheel.
    cfg = MissionConfig(line_max_wheel_delta=1500)
    cfg.line_command_map = {6: (-2000, 4000)}  # vendor-style hard left
    follower = LineFollower(cfg, line_memory=[], line_graph=_DummyGraph())

    target = cfg.line_command_map[6]
    left, right = follower._apply_steer_limit(*target)
    assert left == -1500
    assert right == 1500
    # Once it has been one tick at (-1500, 1500), the next tick can step a
    # further 1500 toward the target.
    follower._prev_left, follower._prev_right = left, right
    left2, right2 = follower._apply_steer_limit(*target)
    assert left2 == -2000  # already at target
    assert right2 == 3000


def test_line_follower_treats_seven_as_center_when_configured() -> None:
    cfg = MissionConfig(line_code_seven_is_center=True)
    follower = LineFollower(cfg, line_memory=[], line_graph=_DummyGraph())

    assert follower.is_line_lost(7) is False
    assert follower._duty_for_ir(7) == cfg.line_command_map[2]


def test_line_follower_pd_outputs_smooth_forward_corrections() -> None:
    cfg = MissionConfig(
        line_base_speed=650,
        line_crawl_speed=300,
        line_pd_kp=150,
        line_pd_kd=0,
        line_max_turn=350,
        line_turn_slowdown=0.2,
        line_min_forward_duty=320,
        line_max_wheel_delta=1000,
    )
    follower = LineFollower(cfg, line_memory=[], line_graph=_DummyGraph())
    ctx = _DummyContext(ir=3, now=1.0)

    follower.step(ctx)

    left, right = ctx.drives[-1]
    assert left > right
    assert left >= 0 and right >= 0
    assert min(left, right) >= cfg.line_min_forward_duty


def test_line_follower_startup_probe_moves_forward_before_sweep() -> None:
    cfg = MissionConfig(
        line_code_seven_is_center=False,
        line_startup_probe_s=1.0,
        line_startup_probe_speed=450,
    )
    follower = LineFollower(cfg, line_memory=[], line_graph=_DummyGraph())
    ctx = _DummyContext(ir=7, now=10.0)

    follower.step(ctx)

    assert ctx.drives[-1] == (450, 450)


def test_line_follower_backtracks_after_losing_seen_line() -> None:
    cfg = MissionConfig(
        line_code_seven_is_center=False,
        line_backtrack_s=1.0,
        line_backtrack_speed=420,
    )
    follower = LineFollower(cfg, line_memory=[], line_graph=_DummyGraph())
    ctx = _DummyContext(ir=2, now=1.0)
    follower.step(ctx)

    ctx.ir = 7
    ctx.now_value = 1.2
    follower.step(ctx)

    # First loss tick brakes toward reverse without snapping past the slew cap.
    first_l, first_r = ctx.drives[-1]
    assert -cfg.line_backtrack_speed <= first_l <= cfg.line_base_speed
    assert -cfg.line_backtrack_speed <= first_r <= cfg.line_base_speed
    ctx.now_value = 1.25
    follower.step(ctx)
    assert ctx.drives[-1][0] < first_l
    assert ctx.drives[-1][1] < first_r


def test_pickup_grace_window_blocks_obstacle_check() -> None:
    cfg = MissionConfig(carry_obstacle_grace_s=1.0)
    pickup = BallPickup(cfg)
    pickup.pick_completed_ts = 100.0
    assert pickup.is_carrying_grace_active(100.5) is True
    assert pickup.is_carrying_grace_active(101.5) is False


class _DummyGraph:
    def add_line_point(self, *_args, **_kwargs) -> None:
        pass

    def add_obstacle(self, *_args, **_kwargs) -> None:
        pass

    def mark_home(self, *_args, **_kwargs) -> None:
        pass


class _DummyContext:
    def __init__(self, *, ir: int, now: float) -> None:
        from challenge.mission.pose import Pose2D

        self.ir = ir
        self.now_value = now
        self.pose = Pose2D()
        self.drives: list[tuple[int, int]] = []
        self.targets: list[tuple[int, int]] = []

    def read_ir(self) -> int:
        return self.ir

    def now(self) -> float:
        return self.now_value

    def drive(self, left: int, right: int) -> None:
        self.drives.append((left, right))

    def set_steer_target(self, left: int, right: int) -> None:
        self.targets.append((left, right))
