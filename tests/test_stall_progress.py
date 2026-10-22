"""Regression: movement along a plan must not count as lack of progress."""

from app.simulation import SimulationEngine


def test_single_robot_long_corridor_not_stalled_completes() -> None:
    rows = ["." * 19]
    eng = SimulationEngine(
        map_rows=rows,
        robot_count=1,
        seed=1,
        auto_orders=False,
        robot_starts=[(0, 0)],
    )
    eng.robots[0].goal = (0, 18)
    eng.robots[0].leg = "debug"
    eng._replan_queued = True
    for _ in range(120):
        eng.step()
        if eng.robots[0].position == (0, 18):
            break
    assert eng.robots[0].position == (0, 18)
    assert eng.metrics.stalls == 0
    assert eng.robots[0].status != "deadlocked"
