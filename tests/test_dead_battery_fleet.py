"""Regression: a depleted robot must not freeze joint validation for the fleet."""

from app.simulation import Command, SimulationEngine


def test_validate_step_allows_dead_robot_to_stay_on_its_cell() -> None:
    eng = SimulationEngine(seed=1, auto_orders=False, robot_count=2)
    eng.debug_set_battery("r1", 0.0)
    nxt = {r.robot_id: r.position for r in eng.robots}
    assert eng._validate_step(nxt) is True


def test_dead_robot_does_not_block_other_robot_from_working() -> None:
    eng = SimulationEngine(seed=1, auto_orders=False, robot_count=2)
    eng.debug_set_battery("r1", 0.0)
    eng.enqueue(Command("add_order", {}))
    for _ in range(300):
        eng.step()
    assert any(
        o.completed_tick is not None and o.picked_tick is not None for o in eng.orders
    )
