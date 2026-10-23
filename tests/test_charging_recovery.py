"""Regression: robots must leave charging and take orders after reaching full charge."""

from app.simulation import Command, SimulationEngine


def test_robot_leaves_charger_and_completes_order_after_full_charge() -> None:
    eng = SimulationEngine(seed=1, auto_orders=False, robot_count=1)
    eng.debug_set_battery("r1", 25.0)
    for _ in range(300):
        eng.step()
    r = eng.robots[0]
    assert r.battery_pct >= 90.0
    eng.enqueue(Command("add_order", {}))
    for _ in range(300):
        eng.step()
    r = eng.robots[0]
    assert r.status != "charging" or r.battery_pct < 90.0
    completed = sum(1 for o in eng.orders if o.completed_tick is not None)
    assert completed >= 1
