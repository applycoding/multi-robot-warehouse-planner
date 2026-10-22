from app.simulation import SimulationEngine


def test_deadlock_counted_once_per_robot() -> None:
    rows = ["....."]
    eng = SimulationEngine(
        map_rows=rows,
        robot_count=2,
        seed=1,
        auto_orders=False,
        robot_starts=[(0, 0), (0, 4)],
    )
    eng.debug_set_goal("r1", (0, 4))
    eng.debug_set_goal("r2", (0, 0))
    for _ in range(400):
        eng.step()
    assert eng.metrics.deadlocks <= 2
