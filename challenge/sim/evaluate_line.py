"""Headless line-follow evaluation for robot-like simulator settings."""

from __future__ import annotations

import argparse
from pathlib import Path
import math

from challenge.mission import MissionConfig
from challenge.mission.config_loader import apply_config_dict, load_config_file
from challenge.sim.arena import Arena
from challenge.sim.runner import SimRunner


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "default.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate line following in the simulator.")
    parser.add_argument(
        "--scenarios",
        default="straight-line,full-course,flicker-ir",
        help="comma-separated scenario names",
    )
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--ticks", type=int, default=500)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--use-vision", action="store_true")
    return parser.parse_args()


def _load_mission_config(path: str) -> MissionConfig:
    cfg = MissionConfig()
    if path:
        apply_config_dict(cfg, load_config_file(Path(path)))
    return cfg


def evaluate_scenario(name: str, *, cfg: MissionConfig, seed: int, ticks: int) -> dict[str, float | int | str]:
    runner = SimRunner(name, seed=seed, config=cfg, use_vision=cfg.use_vision)
    mission = runner.mission
    mission.resume_autonomous()

    line_seen_ticks = 0
    lost_ticks = 0
    max_lost_streak = 0
    current_lost_streak = 0
    max_gap = 0
    total_gap = 0
    max_line_error = 0.0
    total_line_error = 0.0

    try:
        for _ in range(max(0, ticks)):
            runner.tick()
            status = mission.get_status()
            line_error = _distance_to_line_m(
                runner.world.arena,
                runner.world.pose.x_m,
                runner.world.pose.y_m,
            )
            max_line_error = max(max_line_error, line_error)
            total_line_error += line_error
            line_seen = bool(status["line_seen"])
            if line_seen:
                line_seen_ticks += 1
                current_lost_streak = 0
            else:
                lost_ticks += 1
                current_lost_streak += 1
                max_lost_streak = max(max_lost_streak, current_lost_streak)
            gap = int(status["duty_gap"])
            max_gap = max(max_gap, gap)
            total_gap += gap

        status = mission.get_status()
        return {
            "scenario": name,
            "ticks": ticks,
            "line_seen_rate": line_seen_ticks / max(1, ticks),
            "lost_rate": lost_ticks / max(1, ticks),
            "max_lost_streak": max_lost_streak,
            "avg_gap": total_gap / max(1, ticks),
            "max_gap": max_gap,
            "avg_line_error_cm": total_line_error * 100.0 / max(1, ticks),
            "max_line_error_cm": max_line_error * 100.0,
            "x_m": float(runner.world.pose.x_m),
            "y_m": float(runner.world.pose.y_m),
            "home_m": float(status["home_m"]),
            "state": str(status["state"]),
            "watchdog": int(status["watchdog_resets"]),
        }
    finally:
        runner.close()


def _distance_to_line_m(arena: Arena, x: float, y: float) -> float:
    best = float("inf")
    for polyline in arena.line_polylines:
        for start, end in zip(polyline.points_m, polyline.points_m[1:]):
            best = min(best, _distance_to_segment_m(x, y, start, end))
    return best if math.isfinite(best) else 0.0


def _distance_to_segment_m(
    x: float,
    y: float,
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    x0, y0 = start
    x1, y1 = end
    dx = x1 - x0
    dy = y1 - y0
    denom = dx * dx + dy * dy
    if denom <= 1e-12:
        return math.hypot(x - x0, y - y0)
    t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / denom))
    px = x0 + t * dx
    py = y0 + t * dy
    return math.hypot(x - px, y - py)


def main() -> None:
    args = parse_args()
    base_cfg = _load_mission_config(args.config)
    base_cfg.use_vision = bool(args.use_vision or base_cfg.use_vision)

    scenarios = [item.strip() for item in args.scenarios.split(",") if item.strip()]
    print(
        "[line-eval] config=%s loop_sleep=%.3fs line_center=%s crawl=%s"
        % (
            args.config or "dataclass",
            base_cfg.loop_sleep_s,
            base_cfg.line_command_map.get(7),
            base_cfg.line_crawl_speed,
        )
    )
    print(
        "[line-eval] scenario ticks seen lost max_lost avg_gap max_gap "
        "avg_err_cm max_err_cm x y state watchdog"
    )
    failures = 0
    for idx, scenario in enumerate(scenarios):
        cfg = _load_mission_config(args.config)
        cfg.use_vision = base_cfg.use_vision
        row = evaluate_scenario(scenario, cfg=cfg, seed=args.seed + idx, ticks=args.ticks)
        print(
            "[line-eval] {scenario} {ticks} {line_seen_rate:.2f} {lost_rate:.2f} "
            "{max_lost_streak} {avg_gap:.0f} {max_gap} "
            "{avg_line_error_cm:.1f} {max_line_error_cm:.1f} "
            "{x_m:.2f} {y_m:.2f} {state} {watchdog}".format(**row)
        )
        if (
            row["line_seen_rate"] < 0.65
            or row["avg_line_error_cm"] > 8.0
            or row["watchdog"] > 0
        ):
            failures += 1

    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
