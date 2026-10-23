"""Regression: waiting off-goal must cost 1 in low-level search (CBS SOC matches oracle)."""

import random

from app.grid import grid_from_rows
from app.planner.cbs import cbs_plan
from app.planner.validate import validate_joint_plan
from tests.test_oracle import brute_force_soc


def test_wait_cost_siding_two_agent_instance() -> None:
    grid = grid_from_rows(["......", "#.##.#"])
    agents = {"A": ((0, 4), (0, 3)), "B": ((0, 2), (0, 4))}
    optimal = brute_force_soc(grid, agents, allow_wait=True, max_depth=20)
    assert optimal == 8
    res = cbs_plan(
        grid,
        frozenset(),
        agents,
        node_cap=400,
        time_budget_s=0.5,
        max_consecutive_wait=None,
    )
    assert res.reason == "solved"
    assert res.paths is not None
    assert res.soc == optimal
    assert validate_joint_plan(grid, res.paths) == []


def test_cbs_matches_oracle_on_random_tiny_grids() -> None:
    rng = random.Random(42)
    checked = 0
    for seed in range(50):
        rng.seed(seed)
        width = rng.randint(3, 5)
        height = rng.randint(2, 4)
        rows = []
        for _ in range(height):
            row = "".join("." if rng.random() > 0.15 else "#" for _ in range(width))
            if row.count(".") < 2:
                row = "." * width
            rows.append(row)
        grid = grid_from_rows(rows)
        walkable = [
            (r, c)
            for r, row in enumerate(rows)
            for c, ch in enumerate(row)
            if ch == "."
        ]
        if len(walkable) < 4:
            continue
        rng.shuffle(walkable)
        n = rng.randint(2, min(3, len(walkable) // 2))
        agent_ids = [chr(ord("A") + i) for i in range(n)]
        agents: dict[str, tuple[tuple[int, int], tuple[int, int]]] = {}
        for i, aid in enumerate(agent_ids):
            agents[aid] = (walkable[i], walkable[i + n])
        optimal = brute_force_soc(grid, agents, allow_wait=True, max_depth=12)
        if optimal is None:
            continue
        res = cbs_plan(
            grid,
            frozenset(),
            agents,
            node_cap=3000,
            time_budget_s=0.3,
            max_consecutive_wait=None,
        )
        if res.reason != "solved" or res.paths is None:
            continue
        assert validate_joint_plan(grid, res.paths) == []
        assert res.soc == optimal
        checked += 1
    assert checked >= 15
