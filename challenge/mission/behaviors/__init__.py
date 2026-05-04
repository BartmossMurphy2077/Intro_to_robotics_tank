"""Mission behaviors.

Each behavior is a small focused unit (line-follow, pickup, avoidance, seek,
return-home) that talks to the world through a tiny shared interface
(`Behavior`). The controller composes them and decides which one runs each
tick. Adding a new behavior is a matter of writing one file that satisfies
`Behavior` and registering it in `controller.py`.
"""

from .base import Behavior, MissionContext
from .line_follow import LineFollower
from .pickup import BallPickup
from .avoid import ObstacleAvoidance
from .seek_ball import VisionSeeker
from .return_home import ReturnHome

__all__ = [
    "Behavior",
    "MissionContext",
    "LineFollower",
    "BallPickup",
    "ObstacleAvoidance",
    "VisionSeeker",
    "ReturnHome",
]
