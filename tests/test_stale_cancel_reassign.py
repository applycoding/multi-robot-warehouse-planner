"""Regression: orders released from a stall must be reassigned, not same-tick cancelled."""

from app.models import Order
from app.simulation import SimulationEngine


def test_released_order_reassigned_before_stale_cancel() -> None:
    eng = SimulationEngine(seed=2, auto_orders=False, robot_count=2)
    shelf = eng._free_shelf()
    assert shelf is not None
    order = Order(
        order_id="o1",
        shelf_cell=shelf,
        created_tick=0,
        assigned_robot_id="r1",
        unassigned_since_tick=None,
    )
    eng.orders = [order]
    r1, r2 = eng.robots[0], eng.robots[1]
    r1.current_order_id = "o1"
    r1.ticks_without_progress = 50
    r1.hold_streak = 25
    r2.current_order_id = None
    eng.tick = 100

    eng._release_stale_order_assignments()
    assert order.assigned_robot_id is None
    assert order.unassigned_since_tick == 100
    assert order.cancelled_tick is None
    r1.status = "charging"
    r1.leg = "charging"

    eng._run_allocation()
    assert order.assigned_robot_id == "r2"
    assert order.cancelled_tick is None

    eng._cancel_stale_unassigned_orders()
    assert order.cancelled_tick is None
    assert order.assigned_robot_id == "r2"
