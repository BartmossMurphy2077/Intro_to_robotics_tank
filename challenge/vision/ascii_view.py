"""Terminal ASCII art camera viewer.

Run directly as::

    python -m challenge.vision.ascii_view              # local webcam (default)
    python -m challenge.vision.ascii_view --mode sim   # simulator camera
    python -m challenge.vision.ascii_view --mode real  # robot camera

Press Ctrl-C to quit.

Character ramp (dark → bright): " .'`^\",:;Il!i><~+_-?][}{1)(|\\/tfjrxnuvczXYUJCLQ0OZmwqpdbkhao*#MW&8%B@$"
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

# Classic 70-character ramp, reversed so index 0 = darkest (space) and -1 = brightest ($).
# Triple-quoted to safely include both backslash and double-quote characters.
_RAMP_BRIGHT_TO_DARK = """$@B%8&WM#*oahkbdpqwmZO0QLCJUYXzcvunxrjft/\\|()1{}[]?-_+~<>i!lI;:,\",^`'. """
CHARS = _RAMP_BRIGHT_TO_DARK[::-1]  # dark → bright
_N_CHARS = len(CHARS)

# Terminal characters are roughly twice as tall as they are wide.
# Multiply the natural pixel-row count by this factor to avoid vertical stretch.
CHAR_ASPECT = 0.55

# ---------------------------------------------------------------------------
# Core conversion
# ---------------------------------------------------------------------------


def frame_to_ascii(frame_bgr: np.ndarray, cols: int, rows: int, color: bool = True) -> str:
    """Convert a BGR frame to an ASCII string sized to (cols x rows) cells.

    Aspect ratio is preserved (letterboxed vertically).  When *color* is
    True each character is wrapped in a 24-bit ANSI foreground escape code
    drawn from the original pixel colour.
    """
    h, w = frame_bgr.shape[:2]
    if h == 0 or w == 0 or cols <= 0 or rows <= 0:
        return ""

    # How many rows the image would occupy if we used all `cols` columns,
    # corrected for the taller-than-wide nature of terminal cells.
    natural_rows = int(cols * (h / w) * CHAR_ASPECT)
    effective_rows = max(1, min(rows, natural_rows))

    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    small_gray = cv2.resize(gray, (cols, effective_rows), interpolation=cv2.INTER_AREA)

    # Map pixel values [0, 255] → CHARS index; bright → '$', dark → ' '
    indices = (small_gray.astype(np.float32) / 256.0 * _N_CHARS).astype(np.uint8)
    indices = np.clip(indices, 0, _N_CHARS - 1)

    if color:
        # Resize BGR frame to same grid and convert to RGB for ANSI codes
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
            parts.append("\033[0m")  # reset colour at end of each line
            lines.append("".join(parts))
        return "\n".join(lines)
    else:
        lines_plain = ["".join(CHARS[idx] for idx in row) for row in indices]
        return "\n".join(lines_plain)


# ---------------------------------------------------------------------------
# ANSI / terminal helpers
# ---------------------------------------------------------------------------


def _write(s: str) -> None:
    sys.stdout.write(s)


def _flush() -> None:
    sys.stdout.flush()


def _clear_and_home() -> None:
    """Erase the whole screen and move the cursor to the top-left."""
    _write("\033[2J\033[H")


def _cursor_home() -> None:
    """Move the cursor to the top-left without clearing (in-place overwrite)."""
    _write("\033[H")


def _hide_cursor() -> None:
    _write("\033[?25l")


def _show_cursor() -> None:
    _write("\033[?25h")


def get_term_size() -> tuple[int, int]:
    """Return (cols, rows) of the current terminal."""
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
    """Continuously read frames from `get_frame` and render them as ASCII art.

    `get_frame()` must return a BGR ndarray or None (frame skipped).
    Runs until a KeyboardInterrupt (Ctrl-C).
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
                # Reserve one row at the bottom for the status bar
                ascii_rows = max(1, rows - 1)
                art = frame_to_ascii(frame, cols, ascii_rows, color=color)

                h, w = frame.shape[:2]
                status = (
                    f" [ascii-cam]  {w}x{h} → {cols}x{ascii_rows} chars"
                    f"  |  {fps} fps target  |  Ctrl-C to quit"
                )
                # Truncate/pad status to exactly terminal width so prior
                # content on that line is fully overwritten
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
# Camera source factories
# ---------------------------------------------------------------------------


def _make_webcam_source(
    device: int = 0,
) -> tuple[Callable[[], Optional[np.ndarray]], Callable[[], None]]:
    """Open a local webcam and return (get_frame, close)."""
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


def _make_car_source(
    mode: str,
    scenario: str,
) -> tuple[Callable[[], Optional[np.ndarray]], Callable[[], None]]:
    """Build a robot car backend and return (get_frame, close)."""
    from pathlib import Path

    # Ensure the repo root is on sys.path (mirrors main.py)
    repo_root = Path(__file__).resolve().parents[3]
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
        description=(
            "Stream the robot camera (or a local webcam) to the terminal as ASCII art.\n"
            "\n"
            "Character ramp (dark → bright): \" .'`^\\\",:;Il!i><~+_-?][}{1)(|\\\\/tfjrxnuvczXYUJCLQ0OZmwqpdbkhao*#MW&8%B@$\""
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--mode",
        choices=["webcam", "sim", "real", "auto"],
        default="webcam",
        help=(
            "camera source: webcam (default, uses cv2.VideoCapture), "
            "sim (simulator), real (robot hardware), "
            "auto (real on Linux, sim elsewhere)"
        ),
    )
    parser.add_argument(
        "--device",
        type=int,
        default=0,
        help="webcam device index (default 0; only used with --mode webcam)",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=10,
        help="target frames per second (default 10)",
    )
    parser.add_argument(
        "--scenario",
        default="full-course",
        help="sim scenario name (only used with --mode sim or auto)",
    )
    parser.add_argument(
        "--no-color",
        action="store_true",
        help="disable ANSI colour output (plain ASCII only)",
    )
    args = parser.parse_args()

    get_frame: Callable[[], Optional[np.ndarray]]
    close: Callable[[], None]

    if args.mode == "webcam":
        try:
            get_frame, close = _make_webcam_source(args.device)
        except RuntimeError as exc:
            print(f"[ascii-cam] error: {exc}", file=sys.stderr)
            raise SystemExit(1) from None
    else:
        try:
            get_frame, close = _make_car_source(args.mode, args.scenario)
        except RuntimeError as exc:
            print(f"[ascii-cam] error: {exc}", file=sys.stderr)
            raise SystemExit(1) from None

    use_color = not args.no_color
    print(f"[ascii-cam] mode={args.mode}  fps={args.fps}  color={use_color}  Ctrl-C to quit")
    time.sleep(0.4)  # brief pause so the user reads the startup line

    try:
        run_loop(get_frame, fps=args.fps, color=use_color)
    finally:
        close()


if __name__ == "__main__":
    main()
