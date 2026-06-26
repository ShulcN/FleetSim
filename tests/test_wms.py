from backend.wms.fifo_validator import FifoValidator
from backend.wms.orders import Order, OrderStatus, WmsScenario


def make_order(order_id, cargo_type="A"):
    return Order(
        id=order_id,
        release_time=0.0,
        cargo_type=cargo_type,
        pickup_node="P1",
        dropoff_node="D1",
    )


def test_orders_release_by_time():
    scenario = WmsScenario.from_dict(
        {
            "orders": [
                {"id": "late", "release_time": 5.0, "cargo_type": "A", "pickup_node": "P1", "dropoff_node": "D1"},
                {"id": "early", "release_time": 1.0, "cargo_type": "A", "pickup_node": "P1", "dropoff_node": "D1"},
            ]
        }
    )
    # sorted by release_time
    assert [o.id for o in scenario.orders] == ["early", "late"]
    assert scenario.release_due_orders(0.0) == []
    assert [o.id for o in scenario.release_due_orders(1.0)] == ["early"]
    assert [o.id for o in scenario.pending_orders()] == ["early"]
    assert [o.id for o in scenario.release_due_orders(5.0)] == ["late"]


def test_fifo_no_violation_when_in_order():
    fifo = FifoValidator()
    first, second = make_order("A-1"), make_order("A-2")
    fifo.register_released(first)
    fifo.register_released(second)
    assert fifo.register_delivered(first, 1.0) is None
    assert fifo.register_delivered(second, 2.0) is None
    assert fifo.violations == []


def test_fifo_violation_when_out_of_order():
    fifo = FifoValidator()
    first, second = make_order("A-1"), make_order("A-2")
    fifo.register_released(first)
    fifo.register_released(second)
    # deliver the newer order first
    violation = fifo.register_delivered(second, 1.0)
    assert violation is not None
    assert violation.expected_order_id == "A-1"
    assert violation.actual_order_id == "A-2"
