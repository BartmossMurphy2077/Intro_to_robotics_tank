"""Live red-ball detection tester.

Opens the camera (webcam or robot hardware), runs `detect_red_ball` on every
frame, and prints whether a ball is seen — nothing more.  ASCII art and other
visuals live in ascii_view.py; this file is purely about detection.

Usage::

    python -m challenge.vision.detect_live              # webcam (default)
    python -m challenge.vision.detect_live --mode real  # robot camera
    python -m challenge.vision.detect_live --mode sim   # simulator camera

Output::

    [BALL DETECTED]  center=(210, 155)  radius=28.3 px  dist=29.3 cm  frame=640x480
    [no ball]
    [no ball]
    [BALL DETECTED]  center=(214, 158)  radius=29.1 px  dist=28.5 cm  frame=640x480

Press Ctrl-C to quit.
"""

from __future__ import annotations

import sys
import time
from typing import Callable, Optional

import cv2
import numpy as np

from .redball import detect_red_ball


def _get_camera(
    mode: str, device: int, scenario: str
) -> tuple[Callable[[], Optional[np.ndarray]], Callable[[], None]]:
    """Return (get_frame, close) for the requested source."""
    if mode == "webcam":
        cap = cv2.VideoCapture(device)
        if not cap.isOpened():
            raise RuntimeError(
                f"Could not open webcam device {device}. "
                "Make sure a camera is connected and not in use by another application."
            )

        def _get() -> Optional[np.ndarray]:
            ok, frame = cap.read()
            return frame if ok else None

        return _get, cap.release

    # sim / real / auto — use the hardware abstraction layer
    from pathlib import Path

    repo_root = Path(__file__).resolve().parents[2]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))

    from challenge.hardware import make_car

    sim_world = None
    chosen = mode
    if chosen == "auto":
        chosen = "real" if sys.platform.startswith("linux") else "sim"
    if chosen == "sim":
        from challenge.sim.scenarios import build_world
        sim_world = build_world(scenario)

    car = make_car(chosen, world=sim_world)  # type: ignore[arg-type]
    return car.camera.get_frame_bgr, car.close


def run_detection_loop(
    get_frame: Callable[[], Optional[np.ndarray]],
    fps: int = 10,
) -> None:
    """Continuously detect red balls and print results to stdout."""
    interval = 1.0 / max(1, fps)

    print("[detect-live] running — point camera at a red ball  |  Ctrl-C to quit\n")

    prev_detected = False

    try:
        while True:
            t0 = time.monotonic()

            frame = get_frame()
            if frame is not None:
                result = detect_red_ball(frame)

                if result is not None:
                    w, h = result.image_size
                    cx, cy = result.center_xy
                    line = (
                        f"\033[1;32m[BALL DETECTED]\033[0m"
                        f"  center=({cx}, {cy})"
                        f"  radius={result.radius_px:.1f} px"
                        f"  dist={result.distance_cm:.1f} cm"
                        f"  frame={w}x{h}"
                    )
                    print(line)
                    prev_detected = True
                else:
                    # Only print "no ball" once when transitioning, then
                    # keep printing every frame so the output stays live.
                    print("\033[2m[no ball]\033[0m")
                    prev_detected = False

            elapsed = time.monotonic() - t0
            sleep = interval - elapsed
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        pass

    print("\n[detect-live] stopped.")


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(
        description="Live red-ball detection test using the robot or webcam camera.",
    )
    parser.add_argument(
        "--mode", choices=["webcam", "sim", "real", "auto"], default="webcam",
        help="camera source (default: webcam)",
    )
    parser.add_argument("--device", type=int, default=0,
                        help="webcam device index (default 0)")
    parser.add_argument("--fps", type=int, default=10,
                        help="detection rate in frames per second (default 10)")
    parser.add_argument("--scenario", default="full-course",
                        help="sim scenario (only used with --mode sim/auto)")
    args = parser.parse_args()

    try:
        get_frame, close = _get_camera(args.mode, args.device, args.scenario)
    except RuntimeError as exc:
        print(f"[detect-live] error: {exc}", file=sys.stderr)
        raise SystemExit(1) from None

    print(f"[detect-live] mode={args.mode}  fps={args.fps}")
    try:
        run_detection_loop(get_frame, fps=args.fps)
    finally:
        close()


if __name__ == "__main__":
    main()
