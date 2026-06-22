from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import StrEnum
from math import hypot

from backend.maps.dxf_map import DxfMap, Segment2D
from backend.robots.base import RobotBase


class CollisionMode(StrEnum):
    COUNT_ONLY = "count_only"
    STOP_ON_COLLISION = "stop_on_collision"


@dataclass
class CollisionEvent:
    time: float
    type: str  # robot_wall | robot_robot
    robot_id: str
    other_id: str | None = None
    x: float | None = None
    y: float | None = None
    details: str = ""

    def key(self) -> tuple[str, str, str | None]:
        return (self.type, self.robot_id, self.other_id)

    def to_dict(self) -> dict:
        return asdict(self)


class CollisionChecker:
    """Collision system for the MVP.

    Robot geometry is approximated by a conservative circle
    """

    def __init__(self, event_cooldown_s: float = 0.5):
        self.event_cooldown_s = event_cooldown_s
        self._next_allowed_time_by_key: dict[tuple[str, str, str | None], float] = {}

    def check(self, robots: list[RobotBase], dxf_map: DxfMap | None, sim_time: float) -> list[CollisionEvent]:
        raw_events: list[CollisionEvent] = []
        if dxf_map is not None:
            raw_events.extend(self._check_robot_wall(robots, dxf_map, sim_time))
        raw_events.extend(self._check_robot_robot(robots, sim_time))

        events: list[CollisionEvent] = []
        for event in raw_events:
            key = event.key()
            if sim_time >= self._next_allowed_time_by_key.get(key, 0.0):
                self._next_allowed_time_by_key[key] = sim_time + self.event_cooldown_s
                events.append(event)
        return events

    def _check_robot_wall(self, robots: list[RobotBase], dxf_map: DxfMap, sim_time: float) -> list[CollisionEvent]:
        events: list[CollisionEvent] = []
        for robot in robots:
            s = robot.state
            radius = s.collision_radius
            for idx, segment in enumerate(dxf_map.segments):
                dist, closest_x, closest_y = _distance_point_to_segment(s.x, s.y, segment)
                if dist <= radius:
                    events.append(
                        CollisionEvent(
                            time=sim_time,
                            type="robot_wall",
                            robot_id=s.id,
                            other_id=f"wall:{idx}:{segment.layer}",
                            x=closest_x,
                            y=closest_y,
                            details=f"Robot {s.id} touched wall segment on layer {segment.layer}",
                        )
                    )
                    break
        return events

    def _check_robot_robot(self, robots: list[RobotBase], sim_time: float) -> list[CollisionEvent]:
        events: list[CollisionEvent] = []
        for i in range(len(robots)):
            a = robots[i].state
            for j in range(i + 1, len(robots)):
                b = robots[j].state
                dist = hypot(a.x - b.x, a.y - b.y)
                if dist <= a.collision_radius + b.collision_radius:
                    x = (a.x + b.x) * 0.5
                    y = (a.y + b.y) * 0.5
                    events.append(
                        CollisionEvent(
                            time=sim_time,
                            type="robot_robot",
                            robot_id=a.id,
                            other_id=b.id,
                            x=x,
                            y=y,
                            details=f"Robots {a.id} and {b.id} overlap",
                        )
                    )
        return events


def _distance_point_to_segment(px: float, py: float, segment: Segment2D) -> tuple[float, float, float]:
    ax, ay = segment.start.x, segment.start.y
    bx, by = segment.end.x, segment.end.y
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    ab_len_sq = abx * abx + aby * aby

    if ab_len_sq == 0.0:
        return hypot(px - ax, py - ay), ax, ay

    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_len_sq))
    cx = ax + t * abx
    cy = ay + t * aby
    return hypot(px - cx, py - cy), cx, cy
