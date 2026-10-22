from app.grid import grid_from_rows
from app.planner.cbs import cbs_plan
from tests.test_oracle import brute_force_soc


def test_cbs_matches_oracle_with_waits_on_siding_grid() -> None:
    grid = grid_from_rows([".....", "##.##"])
    agents = {
        "A": ((1, 2), (0, 3)),
        "B": ((0, 4), (0, 2)),
        "C": ((0, 3), (0, 0)),
    }
    optimal = brute_force_soc(grid, agents, allow_wait=True)
    res = cbs_plan(
        grid,
        frozenset(),
        agents,
        node_cap=8000,
        time_budget_s=2.0,
        max_consecutive_wait=None,
    )
    assert res.reason == "solved"
    assert res.soc is not None
    assert res.soc <= optimal
