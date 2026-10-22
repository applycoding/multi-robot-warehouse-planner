from __future__ import annotations

from collections import deque

from app.models import Coord, Grid


def brute_force_soc(
    grid: Grid,
    agents: dict[str, tuple[Coord, Coord]],
    max_depth: int = 14,
    allow_wait: bool = True,
    max_consecutive_wait: int | None = None,
) -> int | None:
    ids = sorted(agents.keys())
    starts = {a: agents[a][0] for a in ids}
    goals = {a: agents[a][1] for a in ids}

    def neighbors(state: dict[str, Coord]):
        moves: dict[str, list[Coord]] = {}
        for a in ids:
            opts = grid.neighbors4(state[a])
            if allow_wait or state[a] == goals[a]:
                opts = [state[a]] + opts
            moves[a] = opts
        stacks = [moves[a] for a in ids]

        def rec(i: int, nxt: list[Coord | None]):
            if i == len(ids):
                yield dict(zip(ids, nxt, strict=True))
                return
            for c in stacks[i]:
                nxt[i] = c
                yield from rec(i + 1, nxt)

        for npos in rec(0, [None] * len(ids)):
            cells = list(npos.values())
            if len(cells) != len(set(cells)):
                continue
            ok = True
            for a in ids:
                if abs(npos[a][0] - state[a][0]) + abs(npos[a][1] - state[a][1]) > 1:
                    ok = False
            for i, a in enumerate(ids):
                for b in ids[i + 1 :]:
                    if (
                        npos[a] == state[b]
                        and npos[b] == state[a]
                        and npos[a] != npos[b]
                    ):
                        ok = False
            if ok:
                yield npos

    best: int | None = None
    q: deque[tuple[dict[str, Coord], list[dict[str, Coord]], dict[str, int]]] = deque()
    q.append((starts, [starts], {a: 0 for a in ids}))
    seen: set[tuple[tuple[str, Coord], ...]] = set()
    while q:
        state, hist, wait_run = q.popleft()
        key = tuple(sorted(state.items()))
        if key in seen:
            continue
        seen.add(key)
        if all(state[a] == goals[a] for a in ids):
            soc = 0
            for a in ids:
                arrival = 0
                for i, step in enumerate(hist):
                    if step[a] == goals[a]:
                        arrival = i
                soc += arrival
            if best is None or soc < best:
                best = soc
            continue
        if len(hist) > max_depth:
            continue
        for npos in neighbors(state):
            new_wait: dict[str, int] = {}
            invalid = False
            for a in ids:
                if npos[a] == state[a] and npos[a] != goals[a]:
                    streak = wait_run.get(a, 0) + 1
                    if max_consecutive_wait is not None and streak > max_consecutive_wait:
                        invalid = True
                        break
                    new_wait[a] = streak
                else:
                    new_wait[a] = 0
            if invalid:
                continue
            q.append((npos, hist + [npos], new_wait))
    return best
