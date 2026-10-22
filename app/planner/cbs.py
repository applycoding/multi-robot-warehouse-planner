from __future__ import annotations

import copy
import heapq
import time
from collections.abc import Callable
from dataclasses import dataclass, field

from app.models import Coord, Grid
from app.planner.astar_st import ConstraintTable, path_soc, space_time_astar


def _leave_goal_score(paths: dict[str, list[Coord]], agents: dict[str, tuple[Coord, Coord]]) -> int:
    score = 0
    for aid, (_, goal) in agents.items():
        p = paths.get(aid, [])
        first = None
        for i, c in enumerate(p):
            if c == goal:
                first = i
                break
        if first is None:
            continue
        if any(p[i] != goal for i in range(first + 1, len(p))):
            score += 1
    return score


def _first_arrival_sum(
    paths: dict[str, list[Coord]], agents: dict[str, tuple[Coord, Coord]]
) -> int:
    total = 0
    for aid, (_, goal) in agents.items():
        p = paths.get(aid, [])
        first = len(p)
        for i, c in enumerate(p):
            if c == goal:
                first = i
                break
        total += first
    return total


@dataclass
class Conflict:
    kind: str
    agent_a: str
    agent_b: str
    cell_a: Coord
    cell_b: Coord | None
    k: int


@dataclass
class CBSResult:
    paths: dict[str, list[Coord]] | None
    reason: str
    nodes_expanded: int
    conflicts_resolved: int
    soc: int | None


Clock = Callable[[], float]


def _pad_path(path: list[Coord], length: int) -> list[Coord]:
    if not path:
        return []
    if len(path) >= length:
        return path[:length]
    tail = path[-1]
    return path + [tail] * (length - len(path))


def _find_conflict(
    paths: dict[str, list[Coord]], agents: dict[str, tuple[Coord, Coord]]
) -> Conflict | None:
    ids = sorted(paths.keys())
    max_len = max(len(p) for p in paths.values()) if paths else 0
    padded = {aid: _pad_path(p, max_len) for aid, p in paths.items()}
    for i, a in enumerate(ids):
        for b in ids[i + 1 :]:
            pa, pb = padded[a], padded[b]
            lim = max(len(pa), len(pb))
            for k in range(lim):
                ca = pa[k] if k < len(pa) else pa[-1]
                cb = pb[k] if k < len(pb) else pb[-1]
                if ca == cb:
                    return Conflict("vertex", a, b, ca, cb, k)
                if k > 0:
                    pa_prev = pa[k - 1] if k - 1 < len(pa) else pa[-1]
                    pb_prev = pb[k - 1] if k - 1 < len(pb) else pb[-1]
                    if ca == pb_prev and cb == pa_prev and ca != cb:
                        return Conflict("edge", a, b, pa_prev, pb_prev, k)
    return None


def _compute_soc(paths: dict[str, list[Coord]], agents: dict[str, tuple[Coord, Coord]]) -> int:
    total = 0
    for aid, (start, goal) in agents.items():
        if start == goal:
            continue
        p = paths.get(aid, [])
        total += path_soc(p, goal)
    return total


def _low_level(
    grid: Grid,
    blocked: frozenset[Coord],
    agent: str,
    start: Coord,
    goal: Coord,
    constraints: ConstraintTable,
    horizon: int,
    max_consecutive_wait: int | None,
    deadline: float | None = None,
    clock: Clock = time.monotonic,
) -> list[Coord] | None:
    return space_time_astar(
        grid,
        agent,
        start,
        goal,
        0,
        constraints,
        horizon,
        blocked=blocked,
        max_consecutive_wait=max_consecutive_wait,
        deadline=deadline,
        clock=clock,
    )


@dataclass(order=True)
class _HLNode:
    soc: int
    leave_score: int
    first_arrivals: int
    n_conflicts: int
    counter: int
    constraints: ConstraintTable = field(compare=False)
    paths: dict[str, list[Coord]] = field(compare=False)


_hl_counter = 0


def cbs_plan(
    grid: Grid,
    blocked: frozenset[Coord],
    agents: dict[str, tuple[Coord, Coord]],
    node_cap: int = 400,
    time_budget_s: float = 0.3,
    max_consecutive_wait: int | None = 1,
    clock: Clock = time.monotonic,
) -> CBSResult:
    global _hl_counter
    if not agents:
        return CBSResult({}, "solved", 0, 0, 0)

    start_time = clock()
    deadline = start_time + time_budget_s
    nodes_expanded = 0
    conflicts_resolved = 0

    horizon = grid.free_cell_count() + 5
    root_constraints = ConstraintTable()
    root_paths: dict[str, list[Coord]] = {}
    for aid, (start, goal) in sorted(agents.items()):
        if clock() > deadline:
            return CBSResult(None, "timeout", nodes_expanded, conflicts_resolved, None)
        p = _low_level(
            grid,
            blocked,
            aid,
            start,
            goal,
            root_constraints,
            horizon,
            max_consecutive_wait,
            deadline,
            clock,
        )
        if p is None:
            return CBSResult(None, "low_level_failed", nodes_expanded, conflicts_resolved, None)
        root_paths[aid] = p

    nodes_expanded += 1
    conflict = _find_conflict(root_paths, agents)
    if conflict is None:
        soc = _compute_soc(root_paths, agents)
        return CBSResult(root_paths, "solved", nodes_expanded, conflicts_resolved, soc)

    open_heap: list[tuple[int, int, int, int, _HLNode]] = []
    _hl_counter += 1
    root_node = _HLNode(
        soc=_compute_soc(root_paths, agents),
        leave_score=-_leave_goal_score(root_paths, agents),
        first_arrivals=_first_arrival_sum(root_paths, agents),
        n_conflicts=1,
        counter=_hl_counter,
        constraints=root_constraints,
        paths=root_paths,
    )
    heapq.heappush(
        open_heap,
        (
            root_node.soc,
            root_node.leave_score,
            root_node.first_arrivals,
            root_node.n_conflicts,
            root_node.counter,
            root_node,
        ),
    )

    while open_heap:
        if clock() > deadline:
            return CBSResult(None, "timeout", nodes_expanded, conflicts_resolved, None)

        _, _, _, _, _, node = heapq.heappop(open_heap)
        conflict = _find_conflict(node.paths, agents)
        if conflict is None:
            soc = _compute_soc(node.paths, agents)
            return CBSResult(node.paths, "solved", nodes_expanded, conflicts_resolved, soc)

        if nodes_expanded >= node_cap:
            return CBSResult(None, "node_cap", nodes_expanded, conflicts_resolved, None)

        branches: list[tuple[str, Coord, Coord | None, int]] = []
        if conflict.kind == "vertex":
            branches.append((conflict.agent_a, conflict.cell_a, None, conflict.k))
            cell_b = conflict.cell_b or conflict.cell_a
            branches.append((conflict.agent_b, cell_b, None, conflict.k))
        else:
            a, b = conflict.agent_a, conflict.agent_b
            u, v = conflict.cell_a, conflict.cell_b
            if v is None:
                continue
            branches.append((a, u, v, conflict.k))
            branches.append((b, v, u, conflict.k))

        for agent, u, v, k in branches:
            if clock() > deadline:
                return CBSResult(None, "timeout", nodes_expanded, conflicts_resolved, None)
            if nodes_expanded >= node_cap:
                return CBSResult(None, "node_cap", nodes_expanded, conflicts_resolved, None)

            new_constraints = copy.deepcopy(node.constraints)
            if v is None:
                new_constraints.forbid_vertex(agent, u, k)
            else:
                new_constraints.forbid_edge(agent, u, v, k)

            new_paths = dict(node.paths)
            start, goal = agents[agent]
            p = _low_level(
                grid,
                blocked,
                agent,
                start,
                goal,
                new_constraints,
                horizon,
                max_consecutive_wait,
                deadline,
                clock,
            )
            if p is None:
                nodes_expanded += 1
                continue
            new_paths[agent] = p
            nodes_expanded += 1
            conflicts_resolved += 1

            child_conflict = _find_conflict(new_paths, agents)
            n_conf = 1 if child_conflict else 0
            _hl_counter += 1
            child = _HLNode(
                soc=_compute_soc(new_paths, agents),
                leave_score=-_leave_goal_score(new_paths, agents),
                first_arrivals=_first_arrival_sum(new_paths, agents),
                n_conflicts=n_conf,
                counter=_hl_counter,
                constraints=new_constraints,
                paths=new_paths,
            )
            heapq.heappush(
                open_heap,
                (
                    child.soc,
                    child.leave_score,
                    child.first_arrivals,
                    child.n_conflicts,
                    child.counter,
                    child,
                ),
            )

    return CBSResult(None, "node_cap", nodes_expanded, conflicts_resolved, None)
