"""Per-tick mission telemetry → CSV.

Writes one row per `ChallengeMission.drive()` call so we can reconstruct *why*
each motor command went out (which IR code, which state, what the line
follower wanted before the slew limiter clipped it). Intended for diagnosing
"sits on the line" / "overcorrects" failures where the live HUD changes too
fast to read.

Schema (one row per drive call):

    ts            absolute monotonic time, seconds (float)
    dt_ms         time since previous drive call, milliseconds (int)
    event         "drive" (motor write) or "step" (per-tick snapshot)
    state         FSM state (follow_line / pick_ball / avoid_obstacle / ...)
    state_age_s   how long we've been in this state
    reason        last state-transition reason
    operator      "auto" or "manual"
    ir_raw        3-bit IR code straight off the sensor
    ir_used       3-bit code after auto-invert / majority filter
    ir_inverted   1 if runtime auto-invert is active
    dist_cm       median-filtered ultrasonic distance (cm); -1 = no reading
    carrying      1 if carrying a ball
    target_l      wheel duty the line follower wanted (pre-slew-limit)
    target_r      same, right wheel
    drive_l       wheel duty actually sent to the motor (post-slew-limit)
    drive_r       same, right wheel
    x, y          dead-reckoned pose (m)
    heading_deg   pose heading
    line_lost     consecutive ticks the IR has not seen the line

The file is opened on first write and flushed after every row so a crash
still leaves a usable trace.
"""

from __future__ import annotations

import csv
import os
import time
from pathlib import Path
from typing import Any, TextIO


_FIELDS: tuple[str, ...] = (
    "ts",
    "dt_ms",
    "event",
    "state",
    "state_age_s",
    "reason",
    "operator",
    "ir_raw",
    "ir_used",
    "ir_inverted",
    "dist_cm",
    "carrying",
    "target_l",
    "target_r",
    "drive_l",
    "drive_r",
    "x",
    "y",
    "heading_deg",
    "line_lost",
)


class MotorTelemetry:
    """Append-only CSV writer for drive() and step() events."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh: TextIO | None = None
        self._writer: csv.DictWriter | None = None
        self._last_ts: float | None = None
        self._closed: bool = False

    def _ensure_open(self) -> None:
        if self._fh is not None:
            return
        self._fh = self.path.open("w", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._fh, fieldnames=_FIELDS)
        self._writer.writeheader()
        self._fh.flush()

    def log(self, event: str, snapshot: dict[str, Any]) -> None:
        if self._closed:
            return
        self._ensure_open()
        assert self._writer is not None and self._fh is not None
        ts = float(snapshot.get("ts", time.monotonic()))
        dt_ms = 0 if self._last_ts is None else int(round((ts - self._last_ts) * 1000.0))
        self._last_ts = ts
        row = {
            "ts": f"{ts:.4f}",
            "dt_ms": dt_ms,
            "event": event,
            "state": snapshot.get("state", ""),
            "state_age_s": _round(snapshot.get("state_age_s"), 3),
            "reason": snapshot.get("reason", ""),
            "operator": snapshot.get("operator", ""),
            "ir_raw": snapshot.get("ir_raw", ""),
            "ir_used": snapshot.get("ir_used", ""),
            "ir_inverted": snapshot.get("ir_inverted", ""),
            "dist_cm": _round(snapshot.get("dist_cm"), 1),
            "carrying": snapshot.get("carrying", ""),
            "target_l": snapshot.get("target_l", ""),
            "target_r": snapshot.get("target_r", ""),
            "drive_l": snapshot.get("drive_l", ""),
            "drive_r": snapshot.get("drive_r", ""),
            "x": _round(snapshot.get("x"), 3),
            "y": _round(snapshot.get("y"), 3),
            "heading_deg": _round(snapshot.get("heading_deg"), 1),
            "line_lost": snapshot.get("line_lost", ""),
        }
        self._writer.writerow(row)
        self._fh.flush()

    def close(self) -> None:
        if self._fh is not None:
            try:
                self._fh.flush()
                self._fh.close()
            except Exception:
                pass
        self._fh = None
        self._writer = None
        self._closed = True


def default_telemetry_path(repo_root: Path) -> Path:
    """`outputs/telemetry/run_<unix_ts>.csv` under the repo."""
    ts = int(time.time())
    return repo_root / "outputs" / "telemetry" / f"run_{ts}.csv"


def _round(value: Any, ndigits: int) -> Any:
    if value is None or value == "":
        return ""
    try:
        return round(float(value), ndigits)
    except (TypeError, ValueError):
        return value


__all__ = ["MotorTelemetry", "default_telemetry_path"]
