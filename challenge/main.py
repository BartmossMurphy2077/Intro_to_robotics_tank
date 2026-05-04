"""Challenge entry point.

This file is intentionally thin. It only:
- parses CLI args
- builds a `MissionConfig` and (optionally) a sim world / visualizer
- constructs the car backend (real or sim) via `make_car`
- delegates to either `runtime.loop.run_mission` for the interactive loop
  or `_run_calibration` for one-shot sensor probing.

All behavior logic lives in `challenge.mission.*` and `challenge.runtime.*`.
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from pathlib import Path

repo_root = Path(__file__).resolve().parents[1]
if str(repo_root) not in sys.path:
    sys.path.insert(0, str(repo_root))

from challenge.hardware import make_car  # noqa: E402
from challenge.mission import ChallengeMission, MissionConfig  # noqa: E402
from challenge.runtime import run_mission  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Run challenge mission. Picks the real Freenove tank on a "
        "Raspberry Pi and the in-process simulator everywhere else."
    )
    p.add_argument(
        "--mode", choices=["sim", "real", "auto"], default="auto",
        help="hardware backend: sim (mock+SimWorld), real (Freenove), auto (default)",
    )
    p.add_argument("--scenario", default="full-course",
                   help="named scenario for sim mode (see challenge/sim/scenarios.py)")
    p.add_argument("--gui", action="store_true",
                   help="open the pygame visualizer (sim mode only)")
    p.add_argument("--seed", type=int, default=None,
                   help="seed sim RNG for reproducible runs")
    p.add_argument("--use-vision", action="store_true",
                   help="enable red-ball vision pipeline (mission)")
    p.add_argument(
        "--config", default=str(repo_root / "challenge" / "config" / "default.json"),
        help=(
            "operator-editable mission config JSON (default: "
            "challenge/config/default.json). Pass an empty string to skip "
            "and use the dataclass defaults only."
        ),
    )
    p.add_argument(
        "--use-ga", action="store_true",
        help=(
            "load GA-tuned weights from outputs/ga/best_params.json "
            "(equivalent to --params outputs/ga/best_params.json)"
        ),
    )
    p.add_argument("--params", default=None,
                   help="load mission tuning params JSON (overrides --use-ga)")
    # Argv overrides default to None so the editable config file wins by
    # default; pass them explicitly only when you want to override the file.
    p.add_argument("--obstacle-cm", type=float, default=None)
    p.add_argument("--pickup-cm", type=float, default=None)
    p.add_argument("--home-radius-m", type=float, default=None)
    p.add_argument("--status-interval", type=float, default=1.0)
    p.add_argument("--loop-sleep", type=float, default=None)
    p.add_argument("--line-crawl-speed", type=int, default=None)
    p.add_argument("--ir-zero-lost", action="store_true")
    p.add_argument(
        "--invert-ir",
        action="store_true",
        help="force infrared bit inversion (use if line appears undetected on real robot)",
    )
    p.add_argument("--calibrate", action="store_true",
                   help="print sensor and arm state without running the mission")
    p.add_argument("--calibrate-arm", action="store_true",
                   help="move the carry arm pose during calibration")
    p.add_argument("--calibrate-seconds", type=float, default=8.0)
    p.add_argument("--calibrate-interval", type=float, default=0.75)
    p.add_argument(
        "--ascii-cam",
        action="store_true",
        help="stream the robot camera as ASCII art in this terminal while the mission runs",
    )
    p.add_argument(
        "--ascii-cam-fps",
        type=int,
        default=10,
        help="target frame rate for the ASCII camera viewer (default 10)",
    )
    p.add_argument(
        "--ascii-no-color",
        action="store_true",
        help="disable ANSI colour in the ASCII camera viewer",
    )
    p.add_argument(
        "--telemetry", default=None,
        help=(
            "path to write per-tick motor telemetry CSV "
            "(default: outputs/telemetry/run_<unix_ts>.csv). "
            "Use --no-telemetry to disable."
        ),
    )
    p.add_argument(
        "--no-telemetry", action="store_true",
        help="disable per-tick motor telemetry CSV writing",
    )
    p.add_argument(
        "--auto-start", action="store_true",
        help=(
            "skip the initial MANUAL idle and start in AUTO mode. Useful when "
            "running headless over SSH where there is no terminal to press Q."
        ),
    )
    p.add_argument(
        "--max-seconds", type=float, default=None,
        help=(
            "exit the mission loop after this many seconds (in addition to "
            "Ctrl-C / quit). Useful for bounded headless runs."
        ),
    )
    return p.parse_args()


def _resolve_trained_params_path(args: argparse.Namespace) -> Path | None:
    """Resolve the GA tuning JSON path — opt-in only.

    Order: explicit --params > CHALLENGE_PARAMS env > --use-ga shortcut.
    Returns None if none are set (falling back to the editable config file).
    """
    if getattr(args, "params", None):
        p = Path(args.params)
        return p if p.is_file() else None
    env = os.environ.get("CHALLENGE_PARAMS")
    if env:
        p = Path(env)
        if p.is_file():
            return p
    if getattr(args, "use_ga", False):
        for rel in ("outputs/ga/best_params.json", "best_params.json"):
            p = repo_root / rel
            if p.is_file():
                return p
    return None


def apply_args(cfg: MissionConfig, args: argparse.Namespace) -> None:
    # 1) Editable JSON config (operator defaults). Skip if --config "" or file
    #    missing — the dataclass defaults remain in force.
    config_path_str = (args.config or "").strip()
    if config_path_str:
        config_path = Path(config_path_str)
        if config_path.is_file():
            from challenge.mission.config_loader import (
                apply_config_dict,
                load_config_file,
            )

            data = load_config_file(config_path)
            skipped = apply_config_dict(cfg, data)
            print(f"[challenge] loaded config from {config_path.resolve()}")
            if skipped:
                noisy = [k for k in skipped if not k.startswith("_")]
                if noisy:
                    print(f"[challenge] config: ignored unknown keys: {noisy}")
        else:
            print(f"[challenge] config file not found: {config_path} (using defaults)")

    # 2) Argv overrides — only applied when the operator passed them.
    if args.obstacle_cm is not None:
        cfg.obstacle_distance_cm = args.obstacle_cm
    if args.pickup_cm is not None:
        cfg.pickup_distance_cm = args.pickup_cm
    if args.home_radius_m is not None:
        cfg.home_radius_m = max(0.05, args.home_radius_m)
    if args.loop_sleep is not None:
        cfg.loop_sleep_s = max(0.01, args.loop_sleep)
    if args.line_crawl_speed is not None:
        cfg.line_crawl_speed = max(120, args.line_crawl_speed)
    if args.ir_zero_lost:
        cfg.line_code_zero_is_center = False
    # Keep camera scanning always enabled in mission runtime.
    cfg.use_vision = True
    cfg.vision_every_n_ticks = 1
    if args.invert_ir:
        cfg.ir_invert_bits = True
        cfg.ir_auto_invert_bits = False

    # 3) GA-tuned weights — opt-in via --params, --use-ga, or CHALLENGE_PARAMS.
    trained = _resolve_trained_params_path(args)
    if trained is not None:
        from challenge.tuning import apply_params, load_params

        apply_params(cfg, load_params(trained))
        cfg.trained_params_source = str(trained.resolve())
        print(f"[challenge] loaded tuned params from {cfg.trained_params_source}")


def _build_sim_world(args: argparse.Namespace):
    from challenge.sim.scenarios import build_world

    return build_world(args.scenario, seed=args.seed)


def _run_calibration(
    mission: ChallengeMission,
    cfg: MissionConfig,
    *,
    duration_s: float,
    interval_s: float,
    set_arm_pose: bool,
) -> None:
    servo = getattr(mission.car, "servo", None)
    if set_arm_pose:
        if servo is not None:
            try:
                servo.setServoAngle("0", cfg.carry_servo0_angle)
                servo.setServoAngle("1", cfg.carry_servo1_angle)
            except Exception as exc:
                print(f"[challenge][calibrate] arm move failed: {exc}")
        else:
            print("[challenge][calibrate] arm move skipped: servo unavailable")

    print(f"[challenge][calibrate] seconds={duration_s:.1f} interval={interval_s:.2f} arm={bool(set_arm_pose)}")
    start = time.monotonic()
    while time.monotonic() - start < max(0.0, duration_s):
        s = mission.get_status()
        servo0 = servo1 = None
        if servo is not None:
            try:
                servo0 = servo.getServoAngle("0")
                servo1 = servo.getServoAngle("1")
            except Exception:
                servo0 = servo1 = None
        print(
            f"[challenge][calibrate] state={s['state']} reason={s['state_reason']} "
            f"age={float(s['state_age_s']):.2f}s ir={s['ir']} dist={s['distance_cm']:.1f}cm "
            f"carry={s['carrying']} home={s['home_m']:.2f}m "
            f"servo0={servo0 if servo0 is not None else '-'} "
            f"servo1={servo1 if servo1 is not None else '-'}"
        )
        time.sleep(max(0.05, interval_s))


def main() -> None:
    args = parse_args()
    cfg = MissionConfig()
    apply_args(cfg, args)

    sim_world = None
    chosen_mode = args.mode
    if chosen_mode == "auto":
        chosen_mode = "real" if sys.platform.startswith("linux") else "sim"
    if chosen_mode == "sim":
        sim_world = _build_sim_world(args)

    try:
        car = make_car(chosen_mode, world=sim_world)
    except RuntimeError as exc:
        print(f"[challenge] {exc}", file=sys.stderr)
        raise SystemExit(2) from None

    telemetry = None
    if not args.no_telemetry:
        from challenge.runtime.telemetry import MotorTelemetry, default_telemetry_path

        telemetry_path = Path(args.telemetry) if args.telemetry else default_telemetry_path(repo_root)
        telemetry = MotorTelemetry(telemetry_path)
        print(f"[challenge] telemetry → {telemetry_path}")

    mission = ChallengeMission(car=car, config=cfg, telemetry=telemetry)
    mission.reset_home_anchor()

    if args.calibrate:
        try:
            _run_calibration(
                mission,
                cfg,
                duration_s=args.calibrate_seconds,
                interval_s=args.calibrate_interval,
                set_arm_pose=bool(args.calibrate_arm),
            )
        finally:
            if telemetry is not None:
                try:
                    telemetry.close()
                except Exception:
                    pass
            car.close()
        return

    visualizer = None
    if chosen_mode == "sim" and args.gui:
        try:
            from challenge.sim.visualizer import PygameVisualizer

            visualizer = PygameVisualizer(sim_world, mission)
        except Exception as exc:
            print(f"[challenge] gui disabled: {exc}")
            visualizer = None

    print(f"[challenge] mode={chosen_mode} scenario={args.scenario if chosen_mode == 'sim' else '-'}")

    if args.ascii_cam:
        from challenge.vision.ascii_view import run_loop

        _cam_thread = threading.Thread(
            target=run_loop,
            kwargs={
                "get_frame": car.camera.get_frame_bgr,
                "fps": args.ascii_cam_fps,
                "color": not args.ascii_no_color,
            },
            daemon=True,  # exits automatically when the main process ends
            name="ascii-cam",
        )
        _cam_thread.start()
        print(f"[challenge] ascii-cam started  fps={args.ascii_cam_fps}")

    run_mission(
        mission,
        cfg,
        sim_world=sim_world,
        visualizer=visualizer,
        status_interval_s=max(0.0, args.status_interval),
        auto_start=bool(args.auto_start),
        max_seconds=args.max_seconds,
    )


if __name__ == "__main__":
    main()
