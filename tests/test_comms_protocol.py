import json

from backend.comms.protocol import (
    encode,
    order_message,
    register_message,
    waypoints_from_order,
)
from backend.robots.controllers import Waypoint


def test_encode_is_one_json_line():
    raw = encode({"type": "register", "robot_id": "R1"})
    assert raw.endswith(b"\n")
    assert json.loads(raw.decode("utf-8")) == {"type": "register", "robot_id": "R1"}


def test_register_message():
    assert register_message("R1") == {"type": "register", "robot_id": "R1"}


def test_order_waypoints_round_trip():
    message = order_message("R1", "ORD-1", [Waypoint(1.0, 2.0), Waypoint(3.0, 4.0)])
    assert message["type"] == "order"
    assert message["robot_id"] == "R1"
    assert waypoints_from_order(message) == [Waypoint(1.0, 2.0), Waypoint(3.0, 4.0)]
