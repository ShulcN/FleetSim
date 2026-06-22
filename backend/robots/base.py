from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass, field
from math import hypot, pi
from typing import Any

from .commands import ControlCommand


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle


@dataclass
class RobotState:
    id: str
    x: float
    y: float
    theta: float
    v: float = 0.0
    omega: float = 0.0
    length: float = 0.8
    width: float = 0.45
    collision_radius: float | None = None
    color: str = "#3b82f6"
    model: str = "unknown"
    mode: str = "manual"  # manual | route | idle | collision
    status: str = "idle"  # idle | to_pickup | to_dropoff | manual | collision
    active_order_id: str | None = None
    cargo_type: str | None = None
    target_node: str | None = None
    parameters: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.collision_radius is None:
            # Conservative circle around the robot footprint.
            self.collision_radius = 0.5 * hypot(self.length, self.width)

    def to_dict(self) -> dict:
        return asdict(self)


class KinematicModel(ABC):
    """Base interface for all robot kinematic models."""

    name: str = "abstract"

    @abstractmethod
    def step(self, state: RobotState, command: ControlCommand, dt: float) -> RobotState:
        """Return updated state after dt seconds."""
        raise NotImplementedError


class RobotBase:
    """Robot entity:
    - KinematicModel integrates motion: state + command*dt -> new state.
    - RouteFollower converts route waypoints into a command.
    """

    def __init__(self, state: RobotState, kinematics: KinematicModel, route_follower=None):
        self.state = state
        self.kinematics = kinematics
        self.route_follower = route_follower
        self.state.model = kinematics.name
        self.command = ControlCommand()

    @property
    def is_idle(self) -> bool:
        return self.state.active_order_id is None and self.state.status in {"idle", "manual"} and not self.has_active_route()

    def has_active_route(self) -> bool:
        return bool(self.route_follower is not None and self.route_follower.active)

    def route_completed(self) -> bool:
        return bool(self.route_follower is not None and self.route_follower.completed)

    def acknowledge_route_completed(self) -> None:
        if self.route_follower is not None:
            self.route_follower.completed = False

    def set_command(self, command: ControlCommand) -> None:
        self.command = command
        self.state.mode = "manual"
        self.state.status = "manual"

        if self.route_follower is not None:
            self.route_follower.clear_route()

    def set_route(self, waypoints) -> None:
        if self.route_follower is None:
            raise RuntimeError(f"Robot {self.state.id} has no route follower")

        self.route_follower.set_route(waypoints)
        self.state.mode = "route"

    def clear_task(self) -> None:
        self.state.active_order_id = None
        self.state.cargo_type = None
        self.state.target_node = None
        self.state.status = "idle"
        self.state.mode = "idle"
        self.command = ControlCommand()
        if self.route_follower is not None:
            self.route_follower.clear_route()

    def stop(self) -> None:
        self.command = ControlCommand()
        if self.route_follower is not None:
            self.route_follower.clear_route()
        self.state.mode = "manual"
        self.state.status = "manual"

    def set_collision_status(self) -> None:
        self.command = ControlCommand()
        self.state.v = 0.0
        self.state.omega = 0.0
        self.state.mode = "collision"
        self.state.status = "collision"

    def update(self, dt: float) -> None:
        if self.route_follower is not None and self.route_follower.active:
            command = self.route_follower.compute_command(self.state)
            self.state.mode = "route"
        else:
            command = self.command
            if self.state.mode == "route":
                self.state.mode = "manual"

        self.state = self.kinematics.step(self.state, command, dt)

    def snapshot(self) -> dict:
        data = self.state.to_dict()
        data["route_follower"] = (
            self.route_follower.snapshot() if self.route_follower is not None else None
        )
        return data
