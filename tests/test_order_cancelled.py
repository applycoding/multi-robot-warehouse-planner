"""Orders released by stale-cancel must show as cancelled, not completed."""

from app.models import Order
from app.simulation import SimulationEngine


def test_stale_unassigned_order_reports_cancelled_status() -> None:
    eng = SimulationEngine(seed=1, auto_orders=False, robot_count=1)
    shelf = eng._free_shelf()
    assert shelf is not None
    eng.orders.append(
        Order(order_id="o_test", shelf_cell=shelf, created_tick=0, assigned_robot_id=None)
    )
    eng.tick = 100
    eng._cancel_stale_unassigned_orders()
    o = eng.orders[0]
    assert o.cancelled_tick == 100
    assert o.completed_tick is None
    snap = eng.snapshot()
    assert snap["orders"][0]["status"] == "cancelled"
    assert eng.metrics.snapshot(eng.tick)["orders_cancelled_total"] == 1
