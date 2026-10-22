from __future__ import annotations

import heapq
import time
from collections.abc import Callable
from dataclasses import dataclass, field

from app.models import Coord, Grid

Clock = Callable[[], float]


@dataclass
class ConstraintTable:
    vertex: dict[tuple[str, Coord, int], None] = field(default_factory=dict)
    edge: dict[tuple[str, Coord, Coord, int], None] = field(default_factory=dict)
    goal_latest: dict[tuple[str, Coord], int] = field(default_factory=dict)

    def forbid_vertex(self, agent: str, cell: Coord, k: int) -> None:
        self.vertex[(agent, cell, k)] = None
        key = (agent, cell)
        prev = self.goal_latest.get(key, -1)
        if k > prev:
            self.goal_latest[key] = k

    def forbid_edge(self, agent: str, u: Coord, v: Coord, k: int) -> None:
        self.edge[(agent, u, v, k)] = None

    def latest_goal_constraint(self, agent: str, goal: Coord) -> int:
        return self.goal_latest.get((agent, goal), -1)

    def latest_constraint_time(self, agent: str) -> int:
        latest = -1
        for a, _, k in self.vertex:
            if a == agent:
                latest = max(latest, k)
        for a, _, _, k in self.edge:
            if a == agent:
                latest = max(latest, k)
        return latest


def _arrival_index(path: list[Coord], goal: Coord) -> int:
    for i, c in enumerate(path):
        if c == goal:
            return i
    return len(path) - 1


def space_time_astar(
    grid: Grid,
    agent: str,
    start: Coord,
    goal: Coord,
    start_time: int,
    constraints: ConstraintTable,
    horizon: int,
    blocked: frozenset[Coord] = frozenset(),
    max_consecutive_wait: int | None = None,
    deadline: float | None = None,
    clock: Clock = time.monotonic,
) -> list[Coord] | None:
    """Time-indexed path; index k is tick start_time + k, path[0] == start."""
    if not grid.is_walkable(start) or not grid.is_walkable(goal):
        return None
    if start in blocked or goal in blocked:
        return None

    dist_map = grid.bfs_distances(goal)
    if start not in dist_map:
        return None

    latest_k = constraints.latest_constraint_time(agent)
    max_k = max(latest_k, 0) + grid.free_cell_count()
    if horizon < max_k:
        max_k = horizon

    def h(cell: Coord) -> int:
        return dist_map.get(cell, 10**6)

    open_heap: list[tuple[int, int, int, Coord, int]] = []
    counter = 0
    heapq.heappush(open_heap, (h(start), 0, counter, start, 0))
    counter += 1
    best_g: dict[tuple[Coord, int], int] = {(start, 0): 0}
    parent: dict[tuple[Coord, int], tuple[Coord, int] | None] = {(start, 0): None}
    wait_run: dict[tuple[Coord, int], int] = {(start, 0): 0}
    closed: set[tuple[Coord, int]] = set()

    goal_latest = constraints.latest_goal_constraint(agent, goal)

    def can_remain_at_goal(k: int) -> bool:
        for kk in range(k + 1, max_k + 1):
            if (agent, goal, kk) in constraints.vertex:
                return False
        return True

    expansions = 0
    while open_heap:
        if deadline is not None and clock() > deadline:
            return None
        expansions += 1
        if expansions % 64 == 0 and deadline is not None and clock() > deadline:
            return None
        f, g, _, cell, k = heapq.heappop(open_heap)
        state = (cell, k)
        if state in closed:
            continue
        closed.add(state)

        if cell == goal and k > goal_latest and can_remain_at_goal(k):
            path: list[Coord] = []
            cur: tuple[Coord, int] | None = state
            while cur is not None:
                path.append(cur[0])
                cur = parent[cur]
            path.reverse()
            return path

        if k >= max_k:
            continue

        nk = k + 1
        candidates: list[Coord] = [cell] + grid.neighbors4(cell)
        for nxt in candidates:
            if nxt in blocked and nxt != cell:
                continue
            if not grid.is_walkable(nxt):
                continue
            if (agent, nxt, nk) in constraints.vertex:
                continue
            if nxt != cell and (agent, cell, nxt, nk) in constraints.edge:
                continue
            if (
                max_consecutive_wait is not None
                and nxt == cell
                and cell != goal
                and wait_run.get(state, 0) + 1 > max_consecutive_wait
            ):
                continue
            ng = g + (0 if nxt == cell else 1)
            nstate = (nxt, nk)
            if nstate in closed:
                continue
            prev = best_g.get(nstate)
            if prev is not None and ng >= prev:
                continue
            best_g[nstate] = ng
            parent[nstate] = state
            wait_run[nstate] = wait_run.get(state, 0) + 1 if nxt == cell else 0
            heapq.heappush(open_heap, (ng + h(nxt), ng, counter, nxt, nk))
            counter += 1

    return None


def path_soc(path: list[Coord], goal: Coord) -> int:
    """Arrival offset: index where goal is reached and never left."""
    last_goal = -1
    for i, c in enumerate(path):
        if c == goal:
            last_goal = i
    return last_goal if last_goal >= 0 else 0


def extend_paths_to_horizon(paths: dict[str, list[Coord]], horizon: int) -> dict[str, list[Coord]]:
    out: dict[str, list[Coord]] = {}
    for aid, p in paths.items():
        if not p:
            out[aid] = []
            continue
        if len(p) >= horizon:
            out[aid] = p[:horizon]
        else:
            tail = p[-1]
            out[aid] = p + [tail] * (horizon - len(p))
    return out
