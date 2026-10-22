from app.planner.validate import validate_trajectory
from app.simulation import SimulationEngine


def test_battery_drain_and_charge() -> None:
    eng = SimulationEngine(robot_count=4, seed=2, auto_orders=False)
    r = eng.robots[0]
    eng.debug_set_battery(r.robot_id, 50.0)
    nxt = eng.grid.neighbors4(r.position)[0]
    r.goal = nxt
    r.status = "moving"
    eng.committed_paths = {rb.robot_id: [rb.position] for rb in eng.robots}
    eng.committed_paths[r.robot_id] = [r.position, nxt]
    eng.plan_start_tick = eng.tick
    eng.planner_used = "cbs"
    before = r.battery_pct
    eng.step()
    assert r.battery_pct == before - 0.5

    rows = [
        "#######",
        "#C....#",
        "#H....#",
        "#######",
    ]
    eng2 = SimulationEngine(map_rows=rows, robot_count=1, seed=1, auto_orders=False)
    eng2.robots[0].position = (2, 1)
    eng2.debug_set_battery("r1", 15.0)
    eng2._replan_queued = True
    for _ in range(80):
        eng2.step()
        if eng2.robots[0].battery_pct >= 100.0:
            break
    assert eng2.robots[0].battery_pct >= 100.0
    assert eng2.robots[0].status == "charging"


def test_order_lifecycle() -> None:
    rows = [
        "#######",
        "#P.a..#",
        "#.....#",
        "#H....#",
        "#######",
    ]
    eng = SimulationEngine(map_rows=rows, robot_count=1, seed=1, auto_orders=False)
    eng.robots[0].position = (3, 1)
    shelf = (1, 3)
    eng.orders.append(
        __import__("app.models", fromlist=["Order"]).Order("o1", shelf, 0, "r1")
    )
    eng.robots[0].current_order_id = "o1"
    eng.robots[0].goal = shelf
    eng.robots[0].leg = "to_shelf"
    eng._replan_queued = True
    for _ in range(80):
        eng.step()
        if eng.orders[0].completed_tick is not None:
            break
    assert eng.orders[0].completed_tick is not None


def test_siding_corridor_no_stall() -> None:
    rows = ["#####", ".....", "##.##"]
    eng = SimulationEngine(
        map_rows=rows,
        robot_count=2,
        seed=1,
        auto_orders=False,
        robot_starts=[(1, 0), (1, 4)],
    )
    eng.debug_set_goal("r1", (1, 4))
    eng.debug_set_goal("r2", (1, 0))
    for _ in range(60):
        eng.step()
    assert eng.metrics.stalls == 0
    assert all(r.status != "stalled" for r in eng.robots)


def test_pocketless_swap_hold_and_stall() -> None:
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
    positions_by_tick: dict[int, dict[str, tuple[int, int]]] = {}
    blocked_by_tick: dict[int, set[tuple[int, int]]] = {t: set() for t in range(200)}
    saw_hold = False
    first_stall_tick: int | None = None
    for t in range(200):
        frame = eng.step()
        if frame["planner_used"] == "hold":
            saw_hold = True
        if first_stall_tick is None and eng.metrics.stalls >= 1:
            first_stall_tick = t
        positions_by_tick[t] = {rb["id"]: tuple(rb["pos"]) for rb in frame["robots"]}
    assert saw_hold
    assert eng.metrics.stalls >= 1
    assert first_stall_tick is not None
    assert 14 <= first_stall_tick <= 20
    assert validate_trajectory(eng.grid, positions_by_tick, blocked_by_tick) == []


def test_remove_robot_returns_order() -> None:
    eng = SimulationEngine(robot_count=5, seed=1, auto_orders=False)
    from app.models import Order

    eng.orders.append(Order("o1", (2, 3), 0, "r5"))
    eng.robots[4].current_order_id = "o1"
    eng._set_robot_count(4)
    pending = [o for o in eng.orders if o.assigned_robot_id is None and o.completed_tick is None]
    assert len(pending) == 1
    assert pending[0].order_id == "o1"


def test_battery_zero_blocked() -> None:
    eng = SimulationEngine(robot_count=4, seed=1, auto_orders=False)
    eng.debug_set_battery("r1", 0.0)
    eng.step()
    r1 = next(r for r in eng.robots if r.robot_id == "r1")
    assert r1.status == "blocked"
