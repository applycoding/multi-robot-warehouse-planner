import random

from hypothesis import given, settings
from hypothesis import strategies as st

from app.grid import default_grid, grid_from_rows
from app.planner.cbs import cbs_plan
from app.planner.chain import plan_joint
from app.planner.validate import validate_joint_plan


@settings(max_examples=100, deadline=None, derandomize=True, database=None)
@given(seed=st.integers(0, 10_000), n_agents=st.integers(2, 8))
def test_planner_chain_valid_when_not_hold(seed: int, n_agents: int) -> None:
    grid = default_grid()
    cells = sorted([c for c, t in grid.cell_types.items() if t.value != "wall"])
    endpoints = [c for c in cells if grid.cell_types[c].value != "corridor"]
    rng = random.Random(seed)
    if len(endpoints) < n_agents:
        return
    starts = rng.sample(endpoints, n_agents)
    goals = rng.sample([c for c in endpoints if c not in starts], n_agents)
    if len(goals) < n_agents:
        return
    agents = {f"A{i}": (starts[i], goals[i]) for i in range(n_agents)}
    result = plan_joint(grid, frozenset(), agents, None, seed=seed)
    if result.planner_used == "hold":
        return
    assert validate_joint_plan(grid, result.paths) == []


def test_cbs_soc_small_grids() -> None:
    from tests.test_oracle import brute_force_soc

    grid = grid_from_rows(["....", "...."])
    agents = {"A": ((0, 0), (0, 3)), "B": ((1, 0), (1, 3))}
    res = cbs_plan(grid, frozenset(), agents)
    oracle = brute_force_soc(grid, agents, allow_wait=False)
    assert res.reason == "solved"
    assert res.soc == oracle
