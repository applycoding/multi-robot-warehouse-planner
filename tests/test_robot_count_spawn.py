"""Regression: raising robot count must place each new robot on a distinct parking cell."""

from app.planner.validate import validate_trajectory
from app.simulation import Command, SimulationEngine


def test_raise_robot_count_distinct_parking_cells() -> None:
    eng = SimulationEngine(seed=1, auto_orders=False, robot_count=4)
    eng.enqueue(Command("robot_count", {"count": 8}))
    eng.step()
    positions = [r.position for r in eng.robots]
    assert len(positions) == len(set(positions))
    history = {0: {r.robot_id: r.position for r in eng.robots}}
    assert validate_trajectory(eng.grid, history, {0: set()}) == []
