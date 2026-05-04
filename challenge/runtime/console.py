"""Interactive terminal console for the challenge runtime.

Responsibilities:
- non-blocking read of single-key WASD/space and full-line typed commands
- a clean, single-line status HUD that doesn't fight with the cmd prompt
- graceful no-op fallback on platforms without `termios` (e.g. Windows)
"""

from __future__ import annotations

import select
import sys
from typing import Optional

try:
    import termios
    import tty
except ImportError:  # Windows etc.
    termios = None  # type: ignore[assignment]
    tty = None  # type: ignore[assignment]


_MOVEMENT_KEYS = ("w", "a", "s", "d")


class RuntimeConsole:
    """Streams status lines while keeping a typed `cmd` prompt usable."""

    PROMPT = "[challenge][cmd] "

    def __init__(self) -> None:
        self.enabled = bool(
            termios is not None
            and sys.stdin
            and not sys.stdin.closed
            and sys.stdin.isatty()
        )
        self._fd: Optional[int] = None
        self._term_state = None
        self._buffer = ""

    # ---------- lifecycle ----------

    def start(self) -> None:
        if not self.enabled:
            return
        try:
            self._fd = sys.stdin.fileno()
            self._term_state = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            self._redraw_prompt()
        except Exception:
            self.enabled = False
            self._fd = None
            self._term_state = None

    def stop(self) -> None:
        if self._fd is not None and self._term_state is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._term_state)
            except Exception:
                pass
        if self.enabled:
            sys.stdout.write("\n")
            sys.stdout.flush()

    # ---------- input ----------

    def poll_commands(self) -> list[str]:
        if not self.enabled:
            return self._fallback_poll()

        commands: list[str] = []
        while True:
            try:
                readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            except (OSError, ValueError):
                break
            if not readable:
                break
            char = sys.stdin.read(1)
            if not char:
                break
            command = self._process_char(char)
            if command:
                commands.append(command)
        return commands

    def _fallback_poll(self) -> list[str]:
        if not sys.stdin or sys.stdin.closed or not sys.stdin.isatty():
            return []
        try:
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        except (OSError, ValueError):
            return []
        if not readable:
            return []
        line = sys.stdin.readline()
        if not line:
            return []
        return [line.rstrip("\r\n").lower()]

    def _process_char(self, char: str) -> Optional[str]:
        if char == "\x03":
            raise KeyboardInterrupt

        if char in ("\r", "\n"):
            command = self._buffer.strip().lower()
            self._buffer = ""
            self._redraw_prompt()
            return command if command else None

        if char in ("\x7f", "\b"):
            if self._buffer:
                self._buffer = self._buffer[:-1]
                self._redraw_prompt()
            return None

        lowered = char.lower()
        if not self._buffer and lowered in _MOVEMENT_KEYS:
            return lowered
        if not self._buffer and lowered == "e":
            return "stop"
        if not self._buffer and char == " ":
            return "space"

        if char.isprintable():
            self._buffer += char
            self._redraw_prompt()
        return None

    # ---------- output ----------

    def print_status_line(self, line: str) -> None:
        if not self.enabled:
            print(line)
            return
        sys.stdout.write("\r\033[2K" + line + "\n")
        self._redraw_prompt()

    def print_info_line(self, line: str) -> None:
        self.print_status_line(line)

    def _redraw_prompt(self) -> None:
        if not self.enabled:
            return
        sys.stdout.write("\r\033[2K" + self.PROMPT + self._buffer)
        sys.stdout.flush()


def coalesce_movement_commands(commands: list[str]) -> list[str]:
    """Collapse repeated WASD/space so terminal key-repeat doesn't queue."""
    if not commands:
        return []
    movement = set(_MOVEMENT_KEYS) | {" ", "space"}
    out: list[str] = []
    pending: Optional[str] = None
    for cmd in commands:
        if cmd in movement:
            pending = "space" if cmd == " " else cmd
            continue
        if pending is not None:
            out.append(pending)
            pending = None
        out.append(cmd)
    if pending is not None:
        out.append(pending)
    return out


def format_hud_line(status: dict) -> str:
    """Single concise HUD line. Easy to scan, no extra chrome."""
    mode = "manual" if status.get("manual") else "auto"
    ir_raw = status.get("ir_raw", status.get("ir"))
    ir_inv = status.get("ir_inverted", 0)
    line_seen = status.get("line_seen", 0)
    return (
        f"[{mode}] state={status['state']:<14} "
        f"ir={status['ir']} raw={ir_raw} inv={ir_inv} line={line_seen} "
        f"dist={status['distance_cm']:5.1f}cm "
        f"carry={status['carrying']} home={status['home_m']:.2f}m"
    )


__all__ = ["RuntimeConsole", "coalesce_movement_commands", "format_hud_line"]
