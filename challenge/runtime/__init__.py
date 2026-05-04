"""Runtime helpers for the challenge entry point.

- `console.py`: small terminal HUD + non-blocking input
- `commands.py`: command dispatcher (set/get/setmap/help/status/auto/...)
- `loop.py`: the run loop

`main.py` only assembles these and the mission, so it stays small.
"""

from .console import RuntimeConsole
from .commands import handle_command
from .loop import run_mission

__all__ = ["RuntimeConsole", "handle_command", "run_mission"]
