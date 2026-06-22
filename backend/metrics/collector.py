from __future__ import annotations

from dataclasses import dataclass, field
from math import hypot

from backend.collisions import CollisionEvent
from backend.wms.fifo_validator import FifoViolation
from backend.wms.orders import Order


@dataclass
class MetricsCollector:
    robot_distance_m: dict[str, float] = field(default_factory=dict)
    _last_position: dict[str, tuple[float, float]] = field(default_factory=dict)
    collisions: list[dict] = field(default_factory=list)
    order_events: list[dict] = field(default_factory=list)
    fifo_violations: list[dict] = field(default_factory=list)

    def observe_robots(self, robot_snapshots: list[dict]) -> None:
        for robot in robot_snapshots:
            robot_id = robot["id"]
            x = float(robot["x"])
            y = float(robot["y"])
            prev = self._last_position.get(robot_id)
            if prev is not None:
                self.robot_distance_m[robot_id] = self.robot_distance_m.get(robot_id, 0.0) + hypot(x - prev[0], y - prev[1])
            self._last_position[robot_id] = (x, y)

    def record_collisions(self, events: list[CollisionEvent]) -> None:
        self.collisions.extend(event.to_dict() for event in events)

    def record_order_event(self, time: float, event: str, order: Order, robot_id: str | None = None) -> None:
        self.order_events.append(
            {
                "time": time,
                "event": event,
                "order_id": order.id,
                "cargo_type": order.cargo_type,
                "pickup_node": order.pickup_node,
                "dropoff_node": order.dropoff_node,
                "robot_id": robot_id or order.assigned_robot,
            }
        )

    def record_fifo_violation(self, violation: FifoViolation | None) -> None:
        if violation is not None:
            self.fifo_violations.append(violation.to_dict())

    def to_dict(self, sim_time: float) -> dict:
        delivered = [e for e in self.order_events if e["event"] == "delivered"]
        assigned = [e for e in self.order_events if e["event"] == "assigned"]
        return {
            "sim_time": sim_time,
            "robot_distance_m": self.robot_distance_m,
            "collision_count": len(self.collisions),
            "collisions": self.collisions,
            "orders_assigned": len(assigned),
            "orders_delivered": len(delivered),
            "order_events": self.order_events,
            "fifo_violation_count": len(self.fifo_violations),
            "fifo_violations": self.fifo_violations,
        }
