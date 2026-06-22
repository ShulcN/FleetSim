from __future__ import annotations

from dataclasses import asdict, dataclass

from .orders import Order


@dataclass
class FifoViolation:
    time: float
    cargo_type: str
    expected_order_id: str
    actual_order_id: str
    details: str

    def to_dict(self) -> dict:
        return asdict(self)


class FifoValidator:
    """FIFO Check
    """

    def __init__(self):
        self._arrival_order: dict[str, list[str]] = {}
        self._delivered: set[str] = set()
        self._open_expected_by_type: dict[str, str] = {}
        self.violations: list[FifoViolation] = []

    def register_released(self, order: Order) -> None:
        self._arrival_order.setdefault(order.cargo_type, []).append(order.id)

    def register_delivered(self, order: Order, sim_time: float) -> FifoViolation | None:
        cargo_type = order.cargo_type
        queue = self._arrival_order.setdefault(cargo_type, [])

        expected = next((order_id for order_id in queue if order_id not in self._delivered), None)
        violation: FifoViolation | None = None

        open_expected = self._open_expected_by_type.get(cargo_type)
        if expected is not None and expected != order.id and open_expected is None:
            violation = FifoViolation(
                time=sim_time,
                cargo_type=cargo_type,
                expected_order_id=expected,
                actual_order_id=order.id,
                details=(
                    f"FIFO violation for cargo type {cargo_type}: "
                    f"expected {expected}, delivered {order.id}"
                ),
            )
            self.violations.append(violation)
            self._open_expected_by_type[cargo_type] = expected

        self._delivered.add(order.id)

        if self._open_expected_by_type.get(cargo_type) == order.id:
            self._open_expected_by_type.pop(cargo_type, None)

        return violation

    def to_dict(self) -> dict:
        return {"violations": [v.to_dict() for v in self.violations]}
