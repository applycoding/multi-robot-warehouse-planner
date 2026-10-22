from __future__ import annotations

from app.models import Coord, Grid


def _pad(path: list[Coord], length: int) -> list[Coord]:
    if not path:
        return []
    if len(path) >= length:
        return path[:length]
    tail = path[-1]
    return path + [tail] * (length - len(path))


def validate_joint_plan(
    grid: Grid,
    paths: dict[str, list[Coord]],
    blocked: set[Coord] | None = None,
) -> list[str]:
    violations: list[str] = []
    blocked = blocked or set()
    if not paths:
        return violations

    max_len = max(len(p) for p in paths.values())
    ids = sorted(paths.keys())
    padded = {aid: _pad(p, max_len) for aid, p in paths.items()}

    for aid, p in paths.items():
        if not p:
            violations.append(f"{aid}: empty path")
            continue
        for i in range(1, len(p)):
            a, b = p[i - 1], p[i]
            if abs(a[0] - b[0]) + abs(a[1] - b[1]) > 1:
                violations.append(f"{aid}: invalid step in path at index {i}")

    for k in range(max_len):
        positions: dict[str, Coord] = {aid: padded[aid][k] for aid in ids}
        for aid, c in positions.items():
            if not grid.is_walkable(c):
                violations.append(f"{aid} at tick {k}: cell {c} not walkable")
            if c in blocked:
                violations.append(f"{aid} at tick {k}: cell {c} blocked")

        cells = list(positions.values())
        if len(cells) != len(set(cells)):
            violations.append(f"vertex conflict at tick {k}")

        if k > 0:
            prev = {aid: padded[aid][k - 1] for aid in ids}
            for a in ids:
                ca, pa = positions[a], prev[a]
                if ca != pa:
                    dr = abs(ca[0] - pa[0])
                    dc = abs(ca[1] - pa[1])
                    if dr + dc > 1:
                        violations.append(f"{a} moved more than one cell at tick {k}")
            for i, a in enumerate(ids):
                for b in ids[i + 1 :]:
                    ca, cb = positions[a], positions[b]
                    pa, pb = prev[a], prev[b]
                    if ca == pb and cb == pa and ca != cb:
                        violations.append(f"swap conflict at tick {k} between {a} and {b}")

    return violations


def validate_trajectory(
    grid: Grid,
    positions_by_tick: dict[int, dict[str, Coord]],
    blocked_cells_by_tick: dict[int, set[Coord]],
) -> list[str]:
    violations: list[str] = []
    ticks = sorted(positions_by_tick.keys())
    if not ticks:
        return violations

    for k in ticks:
        blocked = blocked_cells_by_tick.get(k, set())
        positions = positions_by_tick[k]
        for aid, c in positions.items():
            if not grid.is_walkable(c):
                violations.append(f"{aid} at tick {k}: not walkable")
            if c in blocked:
                violations.append(f"{aid} at tick {k}: blocked")

        cells = list(positions.values())
        if len(cells) != len(set(cells)):
            violations.append(f"vertex conflict at tick {k}")

        if k > 0 and (k - 1) in positions_by_tick:
            prev = positions_by_tick[k - 1]
            ids = sorted(positions.keys())
            for a in ids:
                if a not in prev:
                    continue
                ca, pa = positions[a], prev[a]
                if ca != pa:
                    dr = abs(ca[0] - pa[0])
                    dc = abs(ca[1] - pa[1])
                    if dr + dc > 1:
                        violations.append(f"{a} jumped at tick {k}")
            for i, a in enumerate(ids):
                for b in ids[i + 1 :]:
                    if a not in prev or b not in prev:
                        continue
                    ca, cb = positions[a], positions[b]
                    pa, pb = prev[a], prev[b]
                    if ca == pb and cb == pa and ca != cb:
                        violations.append(f"swap at tick {k}")

    return violations
