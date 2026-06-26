from math import pi

from backend.robots.base import RobotState, normalize_angle
from backend.robots.commands import ControlCommand
from backend.robots.differential_drive import DifferentialDriveKinematics


def make_state():
    return RobotState(id="R1", x=0.0, y=0.0, theta=0.0)


def test_moves_forward():
    kin = DifferentialDriveKinematics(max_linear=1.0, max_angular=2.0)
    state = kin.step(make_state(), ControlCommand(linear=1.0, angular=0.0), dt=1.0)
    assert round(state.x, 6) == 1.0
    assert round(state.y, 6) == 0.0
    assert state.v == 1.0


def test_linear_speed_is_clamped():
    kin = DifferentialDriveKinematics(max_linear=1.0, max_angular=2.0)
    state = kin.step(make_state(), ControlCommand(linear=5.0, angular=0.0), dt=1.0)
    assert state.v == 1.0
    assert round(state.x, 6) == 1.0


def test_rotates_in_place():
    kin = DifferentialDriveKinematics(max_linear=1.0, max_angular=2.0)
    state = kin.step(make_state(), ControlCommand(linear=0.0, angular=1.0), dt=1.0)
    assert round(state.theta, 6) == 1.0
    assert round(state.x, 6) == 0.0


def test_normalize_angle_wraps_into_range():
    assert round(normalize_angle(3 * pi / 2), 6) == round(-pi / 2, 6)
    assert round(normalize_angle(-3 * pi / 2), 6) == round(pi / 2, 6)
    assert normalize_angle(0.0) == 0.0
