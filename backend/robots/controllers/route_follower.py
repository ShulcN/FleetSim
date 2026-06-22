from __future__ import annotations

from dataclasses import asdict, dataclass, field
from math import atan2, cos, pi

from backend.robots.base import RobotState, normalize_angle
from backend.robots.commands import ControlCommand


@dataclass
class Waypoint:
    x: float
    y: float

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class RouteFollowerConfig:

    waypoint_tolerance: float = 0.15
    max_linear: float = 0.8
    max_angular: float = 1.4
    k_linear: float = 0.8
    k_angular: float = 2.0
    stop_and_turn_angle: float = pi / 3.0  # 60 deg


@dataclass
class RouteFollower:
    config: RouteFollowerConfig = field(default_factory=RouteFollowerConfig)
    route: list[Waypoint] = field(default_factory=list)
    current_index: int = 0
    active: bool = False
    completed: bool = False

    def set_route(self, waypoints: list[Waypoint]) -> None:
        self.route = waypoints
        self.current_index = 0
        self.active = len(waypoints) > 0
        self.completed = False

    def clear_route(self) -> None:
        self.route = []
        self.current_index = 0
        self.active = False
        self.completed = False

    def compute_command(self, state: RobotState) -> ControlCommand:
        """
        This controller converts a waypoint route into a velocity command.
        It is intentionally simple and kinematic-model independent.
        """

        if not self.active or not self.route:
            return ControlCommand()

        target = self.route[self.current_index]
        dx = target.x - state.x
        dy = target.y - state.y
        distance = (dx * dx + dy * dy) ** 0.5

        while distance <= self.config.waypoint_tolerance:
            self.current_index += 1
            if self.current_index >= len(self.route):
                self.active = False
                self.completed = True
                return ControlCommand()

            target = self.route[self.current_index]
            dx = target.x - state.x
            dy = target.y - state.y
            distance = (dx * dx + dy * dy) ** 0.5

        target_heading = atan2(dy, dx)
        heading_error = normalize_angle(target_heading - state.theta)

        angular = self.config.k_angular * heading_error
        angular = max(-self.config.max_angular, min(self.config.max_angular, angular))

        # If the target is mostly behind or far to the side, rotate first.
        if abs(heading_error) > self.config.stop_and_turn_angle:
            linear = 0.0
        else:
            # Slow down when heading error grows
            heading_factor = max(0.0, cos(heading_error))
            linear = self.config.k_linear * distance * heading_factor
            linear = max(0.0, min(self.config.max_linear, linear))

        return ControlCommand(linear=linear, angular=angular)

    def snapshot(self) -> dict:
        return {
            "active": self.active,
            "completed": self.completed,
            "current_index": self.current_index,
            "route": [p.to_dict() for p in self.route],
        }
