from __future__ import annotations

import random
import time
from collections.abc import Callable
from dataclasses import dataclass

from app.models import Coord, Grid
from app.planner.cbs import CBSResult, cbs_plan
from app.planner.prioritized import prioritized_plan
from app.planner.validate import validate_joint_plan

Clock = Callable[[], float]


@dataclass
class PlanResult:
    paths: dict[str, list[Coord]]
    planner_used: str
    cbs: CBSResult
    planning_ms: float


def _hold_paths(
    grid: Grid,
    blocked: frozenset[Coord],
    agents: dict[str, tuple[Coord, Coord]],
    previous: dict[str, list[Coord]] | None,
) -> dict[str, list[Coord]]:
    if previous is not None:
        trimmed: dict[str, list[Coord]] = {}
        for aid, (start, _) in agents.items():
            path = previous.get(aid, [start])
            if path and path[0] == start:
                trimmed[aid] = path
            elif start in path:
                trimmed[aid] = path[path.index(start) :]
            else:
                trimmed[aid] = [start]
        violations = validate_joint_plan(grid, trimmed, set(blocked))
        if not violations:
            return trimmed
    return {aid: [start] for aid, (start, _) in agents.items()}


def plan_joint(
    grid: Grid,
    blocked: frozenset[Coord],
    agents: dict[str, tuple[Coord, Coord]],
    previous: dict[str, list[Coord]] | None,
    seed: int,
    node_cap: int = 400,
    time_budget_s: float = 0.3,
    clock: Clock = time.monotonic,
) -> PlanResult:
    t0 = clock()
    cbs = cbs_plan(
        grid, blocked, agents, node_cap=node_cap, time_budget_s=time_budget_s, clock=clock
    )
    if cbs.reason == "solved" and cbs.paths is not None:
        planning_ms = (clock() - t0) * 1000.0
        return PlanResult(cbs.paths, "cbs", cbs, planning_ms)

    agent_ids = sorted(agents.keys())
    orders: list[list[str]] = [agent_ids, list(reversed(agent_ids))]
    rng = random.Random(seed)
    for _ in range(3):
        shuffled = list(agent_ids)
        rng.shuffle(shuffled)
        orders.append(shuffled)

    for order in orders:
        if clock() - t0 > time_budget_s:
            break
        pp = prioritized_plan(
            grid,
            blocked,
            agents,
            order,
            time_budget_s=max(0.01, time_budget_s - (clock() - t0)),
            clock=clock,
        )
        if pp is not None and not validate_joint_plan(grid, pp):
            planning_ms = (clock() - t0) * 1000.0
            return PlanResult(pp, "pp", cbs, planning_ms)

    paths = _hold_paths(grid, blocked, agents, previous)
    planning_ms = (clock() - t0) * 1000.0
    return PlanResult(paths, "hold", cbs, planning_ms)
