"""Terminal ASCII art camera viewer — visuals only.

Streams whatever the camera sees as coloured ASCII art directly in the
terminal.  No detection logic lives here; that belongs in detect_live.py.

Usage::

    python -m challenge.vision.ascii_view              # local webcam (default)
    python -m challenge.vision.ascii_view --mode sim   # simulator camera
    python -m challenge.vision.ascii_view --mode real  # robot camera
    python -m challenge.vision.ascii_view --no-color   # plain ASCII, no ANSI colour

Press Ctrl-C to quit.

Character ramp (dark → bright):
    ' .'`^",:;Il!i><~+_-?][}{1)(|\\/tfjrxnuvczXYUJCLQ0OZmwqpdbkhao*#MW&8%B@$'
"""

from __future__ import annotations

import shutil
import sys
import time
from typing import Callable, Optional

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# 70-character ramp: brightest first, then reversed so index 0 = darkest.
_RAMP_BRIGHT_TO_DARK = """$@B%8&WM#*oahkbdpqwmZO0QLCJUYXzcvunxrjft/\\|()1{}[]?-_+~<>i!lI;:,",^`'. """
CHARS = _RAMP_BRIGHT_TO_DARK[::-1]  # dark → bright
_N_CHARS = len(CHARS)

# Terminal chars are ~2× taller than wide — scale rows down to preserve aspect.
CHAR_ASPECT = 0.55

# ---------------------------------------------------------------------------
# Core conversion
# ---------------------------------------------------------------------------


def frame_to_ascii(frame_bgr: np.ndarray, cols: int, rows: int, color: bool = True) -> str:
    """Convert a BGR frame to an ASCII string that fits (cols × rows) cells.

    Aspect ratio is preserved (letterboxed vertically).  When *color* is True
    each character is wrapped in a 24-bit ANSI foreground escape drawn from
    the original pixel colour.
    """
    h, w = frame_bgr.shape[:2]
    if h == 0 or w == 0 or cols <= 0 or rows <= 0:
        return ""

    natural_rows = int(cols * (h / w) * CHAR_ASPECT)
    effective_rows = max(1, min(rows, natural_rows))

    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    small_gray = cv2.resize(gray, (cols, effective_rows), interpolation=cv2.INTER_AREA)

    indices = (small_gray.astype(np.float32) / 256.0 * _N_CHARS).astype(np.uint8)
    indices = np.clip(indices, 0, _N_CHARS - 1)

    if color:
        small_bgr = cv2.resize(frame_bgr, (cols, effective_rows), interpolation=cv2.INTER_AREA)
        small_rgb = cv2.cvtColor(small_bgr, cv2.COLOR_BGR2RGB)
        lines: list[str] = []
        for row_i in range(effective_rows):
            parts: list[str] = []
            idx_row = indices[row_i]
            rgb_row = small_rgb[row_i]
            for col_i in range(cols):
                r, g, b = int(rgb_row[col_i, 0]), int(rgb_row[col_i, 1]), int(rgb_row[col_i, 2])
                parts.append(f"\033[38;2;{r};{g};{b}m{CHARS[idx_row[col_i]]}")
            parts.append("\033[0m")
            lines.append("".join(parts))
        return "\n".join(lines)
    else:
        return "\n".join("".join(CHARS[idx] for idx in row) for row in indices)


# ---------------------------------------------------------------------------
# ANSI helpers
# ---------------------------------------------------------------------------


def _write(s: str) -> None:
    sys.stdout.write(s)


def _flush() -> None:
    sys.stdout.flush()


def _clear_and_home() -> None:
    _write("\033[2J\033[H")


def _cursor_home() -> None:
    _write("\033[H")


def _hide_cursor() -> None:
    _write("\033[?25l")


def _show_cursor() -> None:
    _write("\033[?25h")


def get_term_size() -> tuple[int, int]:
    sz = shutil.get_terminal_size(fallback=(80, 24))
    return sz.columns, sz.lines


# ---------------------------------------------------------------------------
# Display loop
# ---------------------------------------------------------------------------


def run_loop(
    get_frame: Callable[[], Optional[np.ndarray]],
    fps: int = 10,
    color: bool = True,
) -> None:
    """Read frames from *get_frame* and render them as ASCII art.

    Returns when a KeyboardInterrupt (Ctrl-C) is received.
    """
    interval = 1.0 / max(1, fps)
    first = True
    _hide_cursor()
    try:
        while True:
            t0 = time.monotonic()

            frame = get_frame()
            if frame is not None:
                cols, rows = get_term_size()
                ascii_rows = max(1, rows - 1)
                art = frame_to_ascii(frame, cols, ascii_rows, color=color)

                h, w = frame.shape[:2]
                status = (
                    f" [ascii-cam]  {w}x{h} → {cols}x{ascii_rows} chars"
                    f"  |  {fps} fps  |  Ctrl-C to quit"
                )
                status = status[:cols].ljust(cols)

                if first:
                    _clear_and_home()
                    first = False
                else:
                    _cursor_home()

                _write(art + "\n" + status)
                _flush()

            elapsed = time.monotonic() - t0
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        _show_cursor()
        _write("\n")
        _flush()


# ---------------------------------------------------------------------------
# Camera source factories (shared with detect_live.py via import)
# ---------------------------------------------------------------------------


def make_webcam_source(
    device: int = 0,
) -> tuple[Callable[[], Optional[np.ndarray]], Callable[[], None]]:
    """Open a local webcam. Returns (get_frame, close)."""
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        raise RuntimeError(
            f"Could not open webcam device {device}. "
            "Make sure a camera is connected and not in use by another application."
        )

    def get_frame() -> Optional[np.ndarray]:
        ok, frame = cap.read()
        return frame if ok else None

    def close() -> None:
        cap.release()

    return get_frame, close


def make_car_source(
    mode: str,
    scenario: str = "full-course",
) -> tuple[Callable[[], Optional[np.ndarray]], Callable[[], None]]:
    """Build a robot/sim car backend. Returns (get_frame, close)."""
    from pathlib import Path

    repo_root = Path(__file__).resolve().parents[2]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))

    from challenge.hardware import make_car

    sim_world = None
    chosen_mode = mode
    if chosen_mode == "auto":
        chosen_mode = "real" if sys.platform.startswith("linux") else "sim"

    if chosen_mode == "sim":
        from challenge.sim.scenarios import build_world
        sim_world = build_world(scenario)

    car = make_car(chosen_mode, world=sim_world)  # type: ignore[arg-type]

    def get_frame() -> Optional[np.ndarray]:
        return car.camera.get_frame_bgr()

    def close() -> None:
        car.close()

    return get_frame, close


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(
        description="Stream the robot/webcam as ASCII art in the terminal.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--mode", choices=["webcam", "sim", "real", "auto"], default="webcam",
        help="camera source (default: webcam)",
    )
    parser.add_argument("--device", type=int, default=0,
                        help="webcam device index (default 0)")
    parser.add_argument("--fps", type=int, default=10,
                        help="target frames per second (default 10)")
    parser.add_argument("--scenario", default="full-course",
                        help="sim scenario (only used with --mode sim/auto)")
    parser.add_argument("--no-color", action="store_true",
                        help="disable ANSI colour output")
    args = parser.parse_args()

    if args.mode == "webcam":
        try:
            get_frame, close = make_webcam_source(args.device)
        except RuntimeError as exc:
            print(f"[ascii-cam] error: {exc}", file=sys.stderr)
            raise SystemExit(1) from None
    else:
        try:
            get_frame, close = make_car_source(args.mode, args.scenario)
        except RuntimeError as exc:
            print(f"[ascii-cam] error: {exc}", file=sys.stderr)
            raise SystemExit(1) from None

    print(f"[ascii-cam] mode={args.mode}  fps={args.fps}  color={not args.no_color}  Ctrl-C to quit")
    time.sleep(0.4)

    try:
        run_loop(get_frame, fps=args.fps, color=not args.no_color)
    finally:
        close()


if __name__ == "__main__":
    main()
