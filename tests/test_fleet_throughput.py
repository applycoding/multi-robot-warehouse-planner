"""Regression: multi-robot fleet must not gridlock on the default map."""

from app.simulation import SimulationEngine


def _run(seed: int, robot_count: int, ticks: int = 200) -> dict:
    eng = SimulationEngine(robot_count=robot_count, seed=seed, auto_orders=True)
    collisions = 0
    for t in range(ticks):
        frame = eng.step()
        pos = {rb["id"]: tuple(rb["pos"]) for rb in frame["robots"]}
        cells = list(pos.values())
        if len(cells) != len(set(cells)):
            collisions += 1
    created = len(eng.orders)
    completed = sum(1 for o in eng.orders if o.completed_tick is not None)
    m = eng.metrics.snapshot(eng.tick)
    return {
        "collisions": collisions,
        "created": created,
        "completed": completed,
        "stalls": m["stalls"],
        "throughput": m["throughput_per_min"],
        "planner": m["planner_used_counts"],
    }


def test_eight_robots_complete_half_orders_seed_1() -> None:
    r = _run(1, 8)
    assert r["collisions"] == 0
    assert r["created"] > 0
    assert r["completed"] >= r["created"] // 2


def test_eight_robots_complete_half_orders_seed_2() -> None:
    r = _run(2, 8)
    assert r["collisions"] == 0
    assert r["created"] > 0
    assert r["completed"] >= r["created"] // 2


def test_twelve_robots_moves_without_mass_stall() -> None:
    for seed in (1, 2):
        r = _run(seed, 12)
        assert r["collisions"] == 0
        assert r["completed"] > 0
        # metrics.stalls is cumulative over the run, not concurrent stuck robots
        assert r["stalls"] < 24


def _run_extended(
    seed: int, robot_count: int, ticks: int = 400
) -> tuple[SimulationEngine, int, int, int]:
    eng = SimulationEngine(robot_count=robot_count, seed=seed, auto_orders=True)
    collisions = 0
    stuck_dwelling_ticks = 0
    for t in range(ticks):
        frame = eng.step()
        pos = {rb["id"]: tuple(rb["pos"]) for rb in frame["robots"]}
        cells = list(pos.values())
        if len(cells) != len(set(cells)):
            collisions += 1
        for r in eng.robots:
            if r.status == "dwelling" and r.dwell_remaining == 0 and r.leg == "to_station":
                stuck_dwelling_ticks += 1
    second_half = sum(
        1 for o in eng.orders if o.completed_tick is not None and o.completed_tick >= ticks // 2
    )
    return eng, collisions, second_half, stuck_dwelling_ticks


def test_fleet_keeps_completing_orders_second_half_eight_robots() -> None:
    for seed in (1, 2, 3):
        eng, collisions, second_half, stuck_dwelling = _run_extended(seed, 8)
        assert collisions == 0
        assert second_half > 0
        assert stuck_dwelling <= 2


def test_fleet_keeps_completing_orders_second_half_twelve_robots() -> None:
    for seed in (1, 2, 3):
        eng, collisions, second_half, stuck_dwelling = _run_extended(seed, 12)
        assert collisions == 0
        assert second_half > 0
        assert stuck_dwelling <= 2
