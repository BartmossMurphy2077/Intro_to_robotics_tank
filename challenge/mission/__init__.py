"""Challenge mission package.

The mission is composed of:
- `MissionConfig` (`config.py`): tunable parameters
- `MissionState` (`state.py`): high-level state enum
- `Pose2D` (`pose.py`): pose primitives + math
- `LineGraph`, `MapPoint`, `GraphNode` (`memory.py`): spatial memory
- `behaviors/`: focused behavior modules (line-follow, pickup, avoid, seek, return-home)
- `ChallengeMission` (`controller.py`): top-level orchestrator that composes the above

This package layout keeps each behavior testable in isolation and makes it
easy to plug in new ones (just implement `Behavior` and register it on the
controller).
"""

from .config import MissionConfig
from .controller import ChallengeMission
from .memory import GraphNode, LineGraph, MapPoint
from .pose import Pose2D
from .state import MissionState

__all__ = [
    "ChallengeMission",
    "MissionConfig",
    "MissionState",
    "Pose2D",
    "LineGraph",
    "MapPoint",
    "GraphNode",
]
