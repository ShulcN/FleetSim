from __future__ import annotations

import json
from pathlib import Path

from .base import RobotBase, RobotState
from .controllers import RouteFollower, RouteFollowerConfig
from .differential_drive import DifferentialDriveKinematics


def create_robot_from_config(raw: dict) -> RobotBase:
    robot_id = str(raw.get("id") or raw.get("name"))
    robot_type = str(raw.get("type", "differential_drive"))
    pose = raw.get("initial_pose", {})
    footprint = raw.get("footprint", {})
    params = raw.get("parameters", {})
    follower_params = raw.get("route_follower", {})

    if robot_type != "differential_drive":
        raise ValueError(f"Unsupported robot type for MVP: {robot_type}")

    state = RobotState(
        id=robot_id,
        x=float(pose.get("x", 0.0)),
        y=float(pose.get("y", 0.0)),
        theta=float(pose.get("theta", 0.0)),
        length=float(footprint.get("length", params.get("length", 0.8))),
        width=float(footprint.get("width", params.get("width", 0.45))),
        collision_radius=(
            float(footprint["collision_radius"])
            if "collision_radius" in footprint
            else None
        ),
        color=str(raw.get("color", "#3b82f6")),
        parameters=params,
    )

    kinematics = DifferentialDriveKinematics(
        max_linear=float(params.get("max_linear", 1.0)),
        max_angular=float(params.get("max_angular", 1.5)),
    )
    follower = RouteFollower(
        RouteFollowerConfig(
            waypoint_tolerance=float(follower_params.get("waypoint_tolerance", 0.18)),
            max_linear=float(follower_params.get("max_linear", params.get("max_linear", 0.8))),
            max_angular=float(follower_params.get("max_angular", params.get("max_angular", 1.4))),
            k_linear=float(follower_params.get("k_linear", 0.8)),
            k_angular=float(follower_params.get("k_angular", 2.0)),
        )
    )
    return RobotBase(state, kinematics, follower)


def create_robots_from_fleet_dict(data: dict) -> list[RobotBase]:
    return [create_robot_from_config(raw) for raw in data.get("robots", [])]


def load_fleet_from_file(path: str | Path) -> list[RobotBase]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return create_robots_from_fleet_dict(data)


def create_demo_robots() -> list[RobotBase]:
    return create_robots_from_fleet_dict(
        {
            "robots": [
                {
                    "id": "R1",
                    "type": "differential_drive",
                    "color": "#3b82f6",
                    "initial_pose": {"x": 2.0, "y": 2.0, "theta": 0.0},
                    "footprint": {"length": 0.8, "width": 0.45},
                    "parameters": {"max_linear": 1.2, "max_angular": 1.8},
                    "route_follower": {"max_linear": 0.8, "max_angular": 1.6},
                },
                {
                    "id": "R2",
                    "type": "differential_drive",
                    "color": "#ef4444",
                    "initial_pose": {"x": 5.0, "y": 4.0, "theta": 1.57},
                    "footprint": {"length": 0.8, "width": 0.45},
                    "parameters": {"max_linear": 1.0, "max_angular": 1.5},
                    "route_follower": {"max_linear": 0.7, "max_angular": 1.3},
                },
            ]
        }
    )
