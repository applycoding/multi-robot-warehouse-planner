from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class CellType(Enum):
    WALL = "wall"
    CORRIDOR = "corridor"
    SHELF_ACCESS = "shelf_access"
    PICK_STATION = "pick_station"
    CHARGER = "charger"
    PARKING = "parking"


Coord = tuple[int, int]


@dataclass
class Grid:
    width: int
    height: int
    cell_types: dict[Coord, CellType]
    _bfs_cache: dict[tuple[int, tuple[Coord, ...]], dict[Coord, int]] = field(
        default_factory=dict, repr=False, compare=False
    )
    _walkable: list[list[bool]] = field(default_factory=list, repr=False, compare=False)
    _neighbors: dict[Coord, list[Coord]] = field(default_factory=dict, repr=False, compare=False)
    version: int = 0

    def __post_init__(self) -> None:
        if not self._walkable:
            self._walkable = [[False] * self.width for _ in range(self.height)]
            self._neighbors = {}
            for r in range(self.height):
                for c in range(self.width):
                    coord = (r, c)
                    walk = self.cell_types.get(coord) is not CellType.WALL
                    self._walkable[r][c] = walk
            for r in range(self.height):
                for c in range(self.width):
                    if not self._walkable[r][c]:
                        continue
                    coord = (r, c)
                    nbrs: list[Coord] = []
                    if r > 0 and self._walkable[r - 1][c]:
                        nbrs.append((r - 1, c))
                    if r + 1 < self.height and self._walkable[r + 1][c]:
                        nbrs.append((r + 1, c))
                    if c > 0 and self._walkable[r][c - 1]:
                        nbrs.append((r, c - 1))
                    if c + 1 < self.width and self._walkable[r][c + 1]:
                        nbrs.append((r, c + 1))
                    self._neighbors[coord] = nbrs

    def is_walkable(self, c: Coord) -> bool:
        r, col = c
        if 0 <= r < self.height and 0 <= col < self.width:
            return self._walkable[r][col]
        return False

    def neighbors4(self, c: Coord) -> list[Coord]:
        return self._neighbors.get(c, [])

    def free_cell_count(self) -> int:
        return sum(1 for ct in self.cell_types.values() if ct is not CellType.WALL)

    def bfs_distances(
        self, goal: Coord, blocked: frozenset[Coord] | None = None
    ) -> dict[Coord, int]:
        blocked = blocked or frozenset()
        key = (self.version, goal, blocked)
        if key in self._bfs_cache:
            return self._bfs_cache[key]
        if goal in blocked or not self.is_walkable(goal):
            self._bfs_cache[key] = {}
            return {}
        dist: dict[Coord, int] = {goal: 0}
        queue = [goal]
        head = 0
        while head < len(queue):
            cur = queue[head]
            head += 1
            d = dist[cur]
            for n in self.neighbors4(cur):
                if n in blocked or n in dist:
                    continue
                dist[n] = d + 1
                queue.append(n)
        self._bfs_cache[key] = dist
        return dist

    def reachable(self, start: Coord, blocked: set[Coord]) -> set[Coord]:
        if not self.is_walkable(start) or start in blocked:
            return set()
        seen = {start}
        queue = [start]
        head = 0
        while head < len(queue):
            cur = queue[head]
            head += 1
            for n in self.neighbors4(cur):
                if n in blocked or n in seen:
                    continue
                seen.add(n)
                queue.append(n)
        return seen


@dataclass
class Robot:
    robot_id: str
    position: Coord
    battery_pct: float
    status: str
    current_order_id: str | None
    path: list[Coord] = field(default_factory=list)
    path_start_tick: int = 0
    ticks_without_progress: int = 0
    hold_streak: int = 0
    stall_attempts: int = 0
    goal: Coord | None = None
    dwell_remaining: int = 0
    leg: str = "idle"
    waited_this_tick: bool = False


@dataclass
class Order:
    order_id: str
    shelf_cell: Coord
    created_tick: int
    assigned_robot_id: str | None = None
    picked_tick: int | None = None
    completed_tick: int | None = None
    pick_station: Coord | None = None
