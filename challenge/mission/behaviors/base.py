"""Shared interfaces for mission behaviors.

`MissionContext` is the small surface a behavior uses to read the world and
issue motor / state commands. The controller injects itself as the context,
exposing only what behaviors need (drive, sensors, state transitions).
Keeping behaviors blind to the rest of the controller keeps them
substitutable (Liskov) and easy to test in isolation.
"""

from __future__ import annotations

from typing import Optional, Protocol, runtime_checkable

from ..pose import Pose2D
from ..state import MissionState


@runtime_checkable
class MissionContext(Protocol):
    """Minimum surface a behavior needs from the controller."""

    car: object
    pose: Pose2D
    home_pose: Pose2D

    def now(self) -> float: ...
    def sleep(self, seconds: float) -> None: ...
    def drive(self, left: int, right: int) -> None: ...
    def stop_drive(self) -> None: ...
    def distance_cm(self) -> float: ...
    def read_ir(self) -> int: ...
    def is_carrying_ball(self) -> bool: ...
    def set_carrying_ball(self, value: bool) -> None: ...
    def enter_state(self, state: MissionState, reason: Optional[str] = ...) -> None: ...
    def state_age_s(self) -> float: ...
    def remember_obstacle(self, distance_cm: float) -> None: ...
    def remember_ball_here(self) -> None: ...


class Behavior:
    """Base class for behaviors. Subclasses override `step(...)`.

    Behaviors are intentionally stateless re: the controller; they may keep
    their own internal state (e.g. avoidance phase clock) but never reach
    into the controller's internals — they only call methods on
    `MissionContext`.
    """

    name: str = "behavior"

    def step(self, ctx: MissionContext) -> None:  # pragma: no cover - interface
        raise NotImplementedError


__all__ = ["Behavior", "MissionContext"]
