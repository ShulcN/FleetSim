from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from enum import StrEnum
from pathlib import Path


class OrderStatus(StrEnum):
    SCHEDULED = "scheduled"
    RELEASED = "released"
    ASSIGNED = "assigned"
    PICKED = "picked"
    DELIVERED = "delivered"
    FAILED = "failed"


@dataclass
class Order:
    id: str
    release_time: float
    cargo_type: str
    pickup_node: str
    dropoff_node: str
    eligible_robots: list[str] = field(default_factory=list)
    status: OrderStatus = OrderStatus.SCHEDULED
    assigned_robot: str | None = None
    released_at: float | None = None
    assigned_at: float | None = None
    picked_at: float | None = None
    delivered_at: float | None = None

    def to_dict(self) -> dict:
        data = asdict(self)
        data["status"] = str(self.status)
        return data


@dataclass
class WmsScenario:
    orders: list[Order] = field(default_factory=list)
    _next_index: int = 0

    @classmethod
    def from_file(cls, path: str | Path) -> "WmsScenario":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data: dict) -> "WmsScenario":
        orders = []
        for index, raw in enumerate(data.get("orders", [])):
            orders.append(
                Order(
                    id=str(raw.get("id") or f"ORD-{index + 1}"),
                    release_time=float(raw.get("release_time", raw.get("time", 0.0))),
                    cargo_type=str(raw.get("cargo_type", "default")),
                    pickup_node=str(raw.get("pickup_node", raw.get("from", ""))),
                    dropoff_node=str(raw.get("dropoff_node", raw.get("to", ""))),
                    eligible_robots=[str(x) for x in raw.get("eligible_robots", [])],
                )
            )
        orders.sort(key=lambda o: (o.release_time, o.id))
        return cls(orders=orders)

    def release_due_orders(self, sim_time: float) -> list[Order]:
        released: list[Order] = []
        while self._next_index < len(self.orders):
            order = self.orders[self._next_index]
            if order.release_time > sim_time:
                break
            order.status = OrderStatus.RELEASED
            order.released_at = sim_time
            released.append(order)
            self._next_index += 1
        return released

    def pending_orders(self) -> list[Order]:
        return [o for o in self.orders if o.status == OrderStatus.RELEASED]

    def get_order(self, order_id: str) -> Order | None:
        return next((o for o in self.orders if o.id == order_id), None)

    def to_dict(self) -> dict:
        return {"orders": [o.to_dict() for o in self.orders]}
