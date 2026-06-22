from __future__ import annotations

import json
from typing import Any

from backend.robots.base import RobotBase
from backend.robots.controllers import Waypoint


"""
Messages are plain JSON objects. Every message has a "type" field. The transport
sends one JSON object per line.

Message types:
- register: robot send connecting message to fleet manager
- order:    fleet manager tells to robot waypoints to follow
- state:    robot reports its current pose and status to the manager
"""


def encode(message: dict[str, Any]) -> bytes:
    return (json.dumps(message, ensure_ascii=False) + "\n").encode("utf-8")


def register_message(robot_id: str) -> dict[str, Any]:
    return {"type": "register", "robot_id": robot_id}


def order_message(robot_id: str, order_id: str, waypoints: list[Waypoint]) -> dict[str, Any]:
    return {
        "type": "order",
        "robot_id": robot_id,
        "order_id": order_id,
        "waypoints": [{"x": float(wp.x), "y": float(wp.y)} for wp in waypoints],
    }


def waypoints_from_order(message: dict[str, Any]) -> list[Waypoint]:
    return [Waypoint(x=float(p["x"]), y=float(p["y"])) for p in message.get("waypoints", [])]


def state_message(robot: RobotBase) -> dict[str, Any]:
    s = robot.state
    return {
        "type": "state",
        "robot_id": s.id,
        "x": s.x,
        "y": s.y,
        "theta": s.theta,
        "v": s.v,
        "omega": s.omega,
        "status": s.status,
        "active_order_id": s.active_order_id,
    }
