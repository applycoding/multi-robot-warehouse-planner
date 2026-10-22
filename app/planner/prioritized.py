from __future__ import annotations

import time
from collections.abc import Callable

from app.models import Coord, Grid
from app.planner.astar_st import ConstraintTable, space_time_astar
from app.planner.validate import validate_joint_plan

Clock = Callable[[], float]


def _reserve_path_for_agent(
    constraints: ConstraintTable,
    planning_agent: str,
    other_path: list[Coord],
    pad_tail: int,
) -> None:
    if not other_path:
        return
    extended = list(other_path)
    tail = other_path[-1]
    extended.extend([tail] * pad_tail)
    for k, cell in enumerate(extended):
        constraints.forbid_vertex(planning_agent, cell, k)
    for k in range(1, len(extended)):
        u, v = extended[k - 1], extended[k]
        if u != v:
            constraints.forbid_edge(planning_agent, u, v, k)


def prioritized_plan(
    grid: Grid,
    blocked: frozenset[Coord],
    agents: dict[str, tuple[Coord, Coord]],
    priority_order: list[str],
    time_budget_s: float = 0.3,
    clock: Clock = time.monotonic,
) -> dict[str, list[Coord]] | None:
    start = clock()
    deadline = start + time_budget_s
    horizon = grid.free_cell_count() + 5
    pad_tail = grid.free_cell_count()
    paths: dict[str, list[Coord]] = {}
    reserved: list[str] = []

    for aid in priority_order:
        if aid not in agents:
            continue
        start, goal = agents[aid]
        constraints = ConstraintTable()
        for other in reserved:
            _reserve_path_for_agent(constraints, aid, paths[other], pad_tail)
        p = space_time_astar(
            grid,
            aid,
            start,
            goal,
            0,
            constraints,
            horizon,
            blocked=blocked,
            max_consecutive_wait=1,
            deadline=deadline,
            clock=clock,
        )
        if p is None:
            return None
        paths[aid] = p
        reserved.append(aid)

    for aid in agents:
        if aid not in paths:
            start, goal = agents[aid]
            constraints = ConstraintTable()
            for other in reserved:
                _reserve_path_for_agent(constraints, aid, paths[other], pad_tail)
            p = space_time_astar(
                grid,
                aid,
                start,
                goal,
                0,
                constraints,
                horizon,
                blocked=blocked,
                max_consecutive_wait=1,
                deadline=deadline,
                clock=clock,
            )
            if p is None:
                return None
            paths[aid] = p

    if validate_joint_plan(grid, paths):
        return None
    return paths
