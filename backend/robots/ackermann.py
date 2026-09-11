"""Forward-only bicycle model; curvature is specified at the simulated footprint centre."""

from math import cos, sin, isfinite
from .base import KinematicModel, normalize_angle


class AckermannKinematics(KinematicModel):
    name = "ackermann"

    def __init__(self, max_linear=1.0, min_turn_radius=0.8, max_angular=1.5):
        if any(
            not isfinite(x) or x <= 0
            for x in (max_linear, min_turn_radius, max_angular)
        ):
            raise ValueError("Ackermann parameters must be positive and finite")
        self.max_linear = max_linear
        self.min_turn_radius = min_turn_radius
        self.max_angular = max_angular

    def step(self, state, command, dt):
        v = max(0, min(self.max_linear, command.linear))
        cap = min(self.max_angular, v / self.min_turn_radius)
        w = max(-cap, min(cap, command.angular))
        if abs(w) > 1e-12:
            state.x += v / w * (sin(state.theta + w * dt) - sin(state.theta))
            state.y += v / w * (cos(state.theta) - cos(state.theta + w * dt))
        else:
            state.x += v * cos(state.theta) * dt
            state.y += v * sin(state.theta) * dt
        state.theta = normalize_angle(state.theta + w * dt)
        state.v = v
        state.omega = w
        return state
