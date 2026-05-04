"""Mission state enum.

Kept in its own module so behaviors can import it without pulling in the
controller (which would create circular imports).
"""

from __future__ import annotations

from enum import Enum


class MissionState(Enum):
    FOLLOW_LINE = "follow_line"
    SEEK_BALL = "seek_ball"
    AVOID_OBSTACLE = "avoid_obstacle"
    PICK_BALL = "pick_ball"
    RETURN_HOME = "return_home"
    DROP_BALL = "drop_ball"


__all__ = ["MissionState"]
