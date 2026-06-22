from __future__ import annotations

from math import cos, sin

from .base import KinematicModel, RobotState, normalize_angle
from .commands import ControlCommand


class DifferentialDriveKinematics(KinematicModel):
    """
    v = R/2*(wr+wl)
    omega = R/L*(wr-wl).
    """

    name = "differential_drive"

    def __init__(self, max_linear: float = 1.0, max_angular: float = 1.5):
        self.max_linear = max_linear
        self.max_angular = max_angular

    def step(self, state: RobotState, command: ControlCommand, dt: float) -> RobotState:
        v = max(-self.max_linear, min(self.max_linear, command.linear))
        omega = max(-self.max_angular, min(self.max_angular, command.angular))

        state.x += v * cos(state.theta) * dt
        state.y += v * sin(state.theta) * dt
        state.theta = normalize_angle(state.theta + omega * dt)
        state.v = v
        state.omega = omega
        return state
