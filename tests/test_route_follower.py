from backend.robots.base import RobotState
from backend.robots.controllers import RouteFollower, Waypoint


def state_at(x, y, theta=0.0):
    return RobotState(id="R1", x=x, y=y, theta=theta)


def test_no_route_gives_zero_command():
    follower = RouteFollower()
    command = follower.compute_command(state_at(0.0, 0.0))
    assert command.linear == 0.0
    assert command.angular == 0.0


def test_route_completes_at_last_waypoint():
    follower = RouteFollower()
    follower.set_route([Waypoint(0.0, 0.0)])
    assert follower.active is True
    follower.compute_command(state_at(0.0, 0.0))
    assert follower.active is False
    assert follower.completed is True


def test_drives_towards_waypoint_ahead():
    follower = RouteFollower()
    follower.set_route([Waypoint(1.0, 0.0)])
    command = follower.compute_command(state_at(0.0, 0.0, theta=0.0))
    assert command.linear > 0.0
    assert abs(command.angular) < 1e-6


def test_turns_in_place_when_waypoint_is_to_the_side():
    follower = RouteFollower()
    follower.set_route([Waypoint(0.0, 1.0)])
    command = follower.compute_command(state_at(0.0, 0.0, theta=0.0))
    assert command.linear == 0.0
    assert command.angular > 0.0
