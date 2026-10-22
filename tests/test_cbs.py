from app.grid import grid_from_rows
from app.planner.cbs import cbs_plan
from app.planner.chain import plan_joint
from app.planner.prioritized import prioritized_plan


def test_target_conflict_solved() -> None:
    grid = grid_from_rows(["#####", ".....", "##.##"])
    agents = {"A": ((2, 2), (1, 2)), "B": ((1, 0), (1, 4))}
    res = cbs_plan(grid, frozenset(), agents, node_cap=400)
    assert res.reason == "solved"
    assert res.nodes_expanded <= 10
    assert res.paths is not None
    path_a = res.paths["A"]
    idx = next(i for i, c in enumerate(path_a) if c == (1, 2))
    assert any(path_a[i] != (1, 2) for i in range(idx + 1, len(path_a)))


def test_duplicate_goals_fail() -> None:
    grid = grid_from_rows([".....", "....."])
    agents = {"A": ((0, 0), (0, 4)), "B": ((1, 0), (0, 4))}
    res = cbs_plan(grid, frozenset(), agents, node_cap=50)
    assert res.reason in {"node_cap", "low_level_failed"}
    assert res.paths is None


def test_chain_cbs_pp_hold() -> None:
    grid = grid_from_rows(["#####", ".....", "##.##"])
    agents = {"A": ((2, 2), (1, 2)), "B": ((1, 0), (1, 4))}
    r = plan_joint(grid, frozenset(), agents, None, seed=1)
    assert r.planner_used == "cbs"

    grid_j = grid_from_rows(["#####", ".....", "##.##", "##.##"])
    agents_j = {"A": ((1, 0), (1, 2)), "B": ((3, 2), (1, 4))}
    r2 = plan_joint(grid_j, frozenset(), agents_j, None, seed=1, node_cap=1)
    assert r2.planner_used == "pp"

    grid_p = grid_from_rows(["....."])
    agents_p = {"A": ((0, 0), (0, 4)), "B": ((0, 4), (0, 0))}
    r3 = plan_joint(grid_p, frozenset(), agents_p, None, seed=1)
    assert r3.planner_used == "hold"

    assert prioritized_plan(grid_j, frozenset(), agents_j, ["A", "B"]) is None


def test_crossing_soc() -> None:

    grid = grid_from_rows(["...", "...", "..."])
    agents = {"A": ((1, 0), (1, 2)), "B": ((1, 2), (1, 0))}
    res = cbs_plan(grid, frozenset(), agents)
    assert res.reason == "solved"
    assert res.soc == 6


def test_heap_same_soc_no_type_error() -> None:
    grid = grid_from_rows([".......", ".......", ".......", "......."])
    agents = {
        "A": ((0, 0), (0, 6)),
        "B": ((1, 0), (1, 6)),
        "C": ((2, 0), (2, 6)),
    }
    cbs_plan(grid, frozenset(), agents, node_cap=5)


def test_timeout_injected_clock() -> None:
    grid = grid_from_rows([".....", ".....", ".....", "....."])
    agents = {"A": ((0, 0), (3, 4)), "B": ((3, 0), (0, 4))}
    t = 0.0

    def clock() -> float:
        nonlocal t
        t += 1.0
        return t

    res = cbs_plan(
        grid, frozenset(), agents, node_cap=400, time_budget_s=0.3, clock=clock
    )
    assert res.reason == "timeout"
    assert res.nodes_expanded < 400
