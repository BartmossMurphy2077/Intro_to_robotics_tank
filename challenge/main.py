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
    p.add_argument("--params", default=None,
                   help="load mission tuning params JSON (e.g. outputs/ga/best_params.json)")
    p.add_argument("--obstacle-cm", type=float, default=18.0)
    p.add_argument("--pickup-cm", type=float, default=8.0)
    p.add_argument("--home-radius-m", type=float, default=0.22)
    p.add_argument("--status-interval", type=float, default=1.0)
    p.add_argument("--loop-sleep", type=float, default=0.05)
    p.add_argument("--line-crawl-speed", type=int, default=260)
    p.add_argument("--ir-zero-lost", action="store_true")
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
    return p.parse_args()


def apply_args(cfg: MissionConfig, args: argparse.Namespace) -> None:
    cfg.obstacle_distance_cm = args.obstacle_cm
    cfg.pickup_distance_cm = args.pickup_cm
    cfg.home_radius_m = max(0.05, args.home_radius_m)
    cfg.loop_sleep_s = max(0.01, args.loop_sleep)
    cfg.line_crawl_speed = max(120, args.line_crawl_speed)
    if args.ir_zero_lost:
        cfg.line_code_zero_is_center = False
    cfg.use_vision = bool(args.use_vision)
    if getattr(args, "params", None):
        from challenge.tuning import apply_params, load_params

        apply_params(cfg, load_params(args.params))


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

    mission = ChallengeMission(car=car, config=cfg)
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
    )


if __name__ == "__main__":
    main()
