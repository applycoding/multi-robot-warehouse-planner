from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any

from app import allocator as alloc_mod
from app.battery import (
    CHARGE_COMPLETE_THRESHOLD,
    LOW_BATTERY_THRESHOLD,
    apply_charge,
    apply_move_drain,
)
from app.grid import (
    DEFAULT_MAP,
    cells_of_type,
    grid_from_rows,
    is_corridor,
    is_endpoint_cell,
)
from app.metrics import MetricsCollector
from app.models import CellType, Coord, Order, Robot
from app.planner.astar_st import path_soc
from app.planner.chain import plan_joint


@dataclass
class Command:
    kind: str
    payload: dict[str, Any] = field(default_factory=dict)


class SimulationEngine:
    def __init__(
        self,
        map_rows: list[str] = DEFAULT_MAP,
        robot_count: int = 6,
        seed: int = 1,
        allocator: str = "greedy",
        auto_orders: bool = True,
        robot_starts: list[Coord] | None = None,
    ) -> None:
        self.map_rows = list(map_rows)
        self.grid = grid_from_rows(self.map_rows)
        self.seed = seed
        self.allocator_name = allocator
        self.auto_orders = auto_orders
        self.robot_count = robot_count
        self.tick = 0
        self.playing = True
        self.speed = 1.0
        self.blocked_cells: set[Coord] = set()
        self.orders: list[Order] = []
        self.robots: list[Robot] = []
        self._order_seq = 0
        self._command_queue: list[Command] = []
        self.metrics = MetricsCollector()
        self.planner_used = "cbs"
        self.committed_paths: dict[str, list[Coord]] = {}
        self.plan_start_tick = 0
        self._replan_queued = True
        self._rng_seed = seed
        self._init_robots(robot_starts)
        self._replan_queued = True

    def _init_robots(self, robot_starts: list[Coord] | None) -> None:
        self.robots = []
        parking = cells_of_type(self.grid, CellType.PARKING)
        if robot_starts:
            starts = list(robot_starts)
        else:
            starts = parking[: self.robot_count]
        for i in range(self.robot_count):
            pos = starts[i] if i < len(starts) else parking[i % len(parking)]
            self.robots.append(
                Robot(
                    robot_id=f"r{i + 1}",
                    position=pos,
                    battery_pct=100.0,
                    status="idle",
                    current_order_id=None,
                )
            )

    def enqueue(self, command: Command) -> None:
        self._command_queue.append(command)

    def process_pending_commands(self) -> None:
        if not self._command_queue:
            return
        self._apply_commands()
        self._command_queue.clear()

    def debug_set_goal(self, robot_id: str, cell: Coord) -> None:
        for r in self.robots:
            if r.robot_id == robot_id:
                r.goal = cell
                r.leg = "debug"
                self._replan_queued = True

    def debug_set_battery(self, robot_id: str, pct: float) -> None:
        for r in self.robots:
            if r.robot_id == robot_id:
                r.battery_pct = pct
                if pct <= 0:
                    r.status = "blocked"
                    self._replan_queued = True

    def _next_order_id(self) -> str:
        self._order_seq += 1
        return f"o{self._order_seq}"

    def _targeted_shelves(self) -> set[Coord]:
        out: set[Coord] = set()
        for o in self.orders:
            if o.completed_tick is None:
                out.add(o.shelf_cell)
        return out

    def _free_shelf(self) -> Coord | None:
        shelves = cells_of_type(self.grid, CellType.SHELF_ACCESS)
        targeted = self._targeted_shelves()
        import random

        rng = random.Random(self.seed + self.tick + self._order_seq)
        candidates = [s for s in shelves if s not in targeted]
        if not candidates:
            return None
        return rng.choice(candidates)

    def _apply_commands(self) -> None:
        for cmd in self._command_queue:
            if cmd.kind == "add_order":
                shelf = self._free_shelf()
                if shelf:
                    self.orders.append(
                        Order(
                            order_id=self._next_order_id(),
                            shelf_cell=shelf,
                            created_tick=self.tick,
                        )
                    )
                    self._replan_queued = True
            elif cmd.kind == "block_cell":
                row, col, blocked = cmd.payload["row"], cmd.payload["col"], cmd.payload["blocked"]
                self._try_block_cell((row, col), blocked)
            elif cmd.kind == "robot_count":
                self._set_robot_count(cmd.payload["count"])
            elif cmd.kind == "control":
                if cmd.payload.get("action") == "play":
                    self.playing = True
                elif cmd.payload.get("action") == "pause":
                    self.playing = False
                if cmd.payload.get("speed") is not None:
                    self.speed = float(cmd.payload["speed"])
            elif cmd.kind == "config":
                if cmd.payload.get("allocator"):
                    self.allocator_name = cmd.payload["allocator"]
                if cmd.payload.get("seed") is not None:
                    self.seed = int(cmd.payload["seed"])
                if cmd.payload.get("auto_orders") is not None:
                    self.auto_orders = bool(cmd.payload["auto_orders"])
            elif cmd.kind == "reset":
                self.__init__(
                    map_rows=self.map_rows,
                    robot_count=self.robot_count,
                    seed=self.seed,
                    allocator=self.allocator_name,
                    auto_orders=self.auto_orders,
                )
        self._command_queue.clear()

    def _try_block_cell(self, cell: Coord, blocked: bool) -> str | None:
        if not blocked:
            self.blocked_cells.discard(cell)
            self.grid.version += 1
            self._replan_queued = True
            return None
        if not is_corridor(self.grid, cell):
            return "not corridor"
        for r in self.robots:
            if r.position == cell:
                return "occupied"
        for r in self.robots:
            nxt = self._robot_next_cell(r)
            if nxt == cell:
                return "committed next"
        if is_endpoint_cell(self.grid, cell):
            return "endpoint"
        trial = set(self.blocked_cells)
        trial.add(cell)
        if not self._connectivity_ok(trial):
            return "disconnect"
        self.blocked_cells.add(cell)
        self.grid.version += 1
        self._replan_queued = True
        return None

    def _connectivity_ok(self, blocked: set[Coord]) -> bool:
        operational = {
            CellType.CORRIDOR,
            CellType.SHELF_ACCESS,
            CellType.PICK_STATION,
            CellType.CHARGER,
            CellType.PARKING,
        }
        cells = [
            c
            for c, t in self.grid.cell_types.items()
            if t in operational and c not in blocked
        ]
        if not cells:
            return False
        seen = self.grid.reachable(cells[0], blocked)
        return all(c in seen for c in cells)

    def try_block_cell_api(self, row: int, col: int, blocked: bool) -> str | None:
        """Return error string or None on success (queued)."""
        cell = (row, col)
        if not blocked:
            self.enqueue(Command("block_cell", {"row": row, "col": col, "blocked": False}))
            return None
        if not is_corridor(self.grid, cell):
            return "not corridor"
        for r in self.robots:
            if r.position == cell:
                return "occupied"
        for r in self.robots:
            if self._robot_next_cell(r) == cell:
                return "committed next"
        if is_endpoint_cell(self.grid, cell):
            return "endpoint"
        trial = set(self.blocked_cells) | {cell}
        if not self._connectivity_ok(trial):
            return "disconnect"
        self.enqueue(Command("block_cell", {"row": row, "col": col, "blocked": True}))
        return None

    def _set_robot_count(self, count: int) -> None:
        if count < 4 or count > 12:
            return
        if count < len(self.robots):
            for r in self.robots[count:]:
                if r.current_order_id:
                    for o in self.orders:
                        if o.order_id == r.current_order_id and o.completed_tick is None:
                            o.assigned_robot_id = None
                            break
            self.robots = self.robots[:count]
        else:
            parking = cells_of_type(self.grid, CellType.PARKING)
            used = {r.position for r in self.robots}
            free = [p for p in parking if p not in used]
            base = len(self.robots)
            for j, i in enumerate(range(base, count)):
                pos = free[j] if j < len(free) else parking[0]
                self.robots.append(
                    Robot(
                        robot_id=f"r{i + 1}",
                        position=pos,
                        battery_pct=100.0,
                        status="idle",
                        current_order_id=None,
                    )
                )
        self.robot_count = count
        self._replan_queued = True

    def _robot_next_cell(self, robot: Robot) -> Coord | None:
        if robot.status in ("blocked", "deadlocked"):
            return robot.position
        rel = self.tick - robot.path_start_tick
        path = self.committed_paths.get(robot.robot_id, [])
        if rel + 1 < len(path):
            return path[rel + 1]
        if path:
            return path[-1]
        return robot.position

    def _static_blocked(self) -> frozenset[Coord]:
        extra = set(self.blocked_cells)
        for r in self.robots:
            if r.status in ("blocked",) or r.battery_pct <= 0:
                extra.add(r.position)
        return frozenset(extra)

    def _is_stationary_for_planning(self, robot: Robot) -> bool:
        if robot.status in ("blocked", "deadlocked", "charging"):
            return True
        if robot.dwell_remaining > 0:
            return True
        if (
            not robot.current_order_id
            and robot.leg in ("idle", "parking")
            and (robot.goal is None or robot.position == robot.goal)
        ):
            return True
        return False

    def _planning_context(self) -> tuple[dict[str, tuple[Coord, Coord]], frozenset[Coord]]:
        blocked = set(self.blocked_cells)
        agents: dict[str, tuple[Coord, Coord]] = {}
        for r in self.robots:
            if r.battery_pct <= 0 or r.status == "blocked":
                blocked.add(r.position)
                agents[r.robot_id] = (r.position, r.position)
                continue
            if self._is_stationary_for_planning(r):
                blocked.add(r.position)
                continue
            if r.status == "deadlocked":
                agents[r.robot_id] = (r.position, r.position)
                continue
            goal = r.goal if r.goal is not None else r.position
            agents[r.robot_id] = (r.position, goal)
        return agents, frozenset(blocked)

    def _replan_time_budget_s(self) -> float:
        extra = max(0, self.robot_count - 4)
        return min(1.5, 0.3 + 0.1 * extra)

    def _assign_goals(self) -> dict[str, tuple[Coord, Coord]]:
        agents: dict[str, tuple[Coord, Coord]] = {}
        for r in self.robots:
            goal = r.goal if r.goal is not None else r.position
            if r.status in ("deadlocked", "blocked") or r.battery_pct <= 0:
                agents[r.robot_id] = (r.position, r.position)
            else:
                agents[r.robot_id] = (r.position, goal)
        return agents

    def _regoal_unreachable(self) -> None:
        blocked = set(self.blocked_cells)
        for r in self.robots:
            if r.status in ("deadlocked", "blocked"):
                continue
            goal = r.goal
            if goal is None:
                continue
            reach = self.grid.reachable(r.position, blocked)
            if goal not in reach:
                if r.current_order_id:
                    for o in self.orders:
                        if o.order_id == r.current_order_id:
                            o.assigned_robot_id = None
                            o.pick_station = None
                            o.picked_tick = None
                            r.current_order_id = None
                            break
                pocket = self._nearest_parking(r.position, blocked)
                r.goal = pocket
                r.leg = "parking"
                self._replan_queued = True

    def _nearest_parking(self, pos: Coord, blocked: set[Coord]) -> Coord:
        parking = cells_of_type(self.grid, CellType.PARKING)
        reserved = {r.goal for r in self.robots if r.goal}
        reach = self.grid.reachable(pos, blocked)
        best: Coord | None = None
        best_d = 10**6
        dist_from = self.grid.bfs_distances(pos)
        for p in parking:
            if p not in reach:
                continue
            if p in reserved and p != pos:
                continue
            d = dist_from.get(p, 10**6)
            if d < best_d or (d == best_d and best is not None and p < best):
                best_d = d
                best = p
        return best if best is not None else pos

    def _run_allocation(self) -> None:
        if self.allocator_name == "optimal":
            assignments = alloc_mod.optimal_assign(self.orders, self.robots, self.grid)
        else:
            assignments = alloc_mod.greedy_assign(self.orders, self.robots, self.grid)
        for order_id, robot_id in assignments.items():
            for o in self.orders:
                if o.order_id == order_id:
                    o.assigned_robot_id = robot_id
                    for r in self.robots:
                        if r.robot_id == robot_id:
                            r.current_order_id = order_id
                            r.goal = o.shelf_cell
                            r.leg = "to_shelf"
                            r.status = "moving"
                            self._replan_queued = True

    def _update_idle_goals(self) -> None:
        blocked = set(self.blocked_cells)
        for r in self.robots:
            if r.status in ("deadlocked", "blocked"):
                continue
            if r.current_order_id:
                continue
            if r.battery_pct < LOW_BATTERY_THRESHOLD and r.leg != "charging":
                ch = self._free_charger(r.position, blocked)
                if ch:
                    r.goal = ch
                    r.leg = "to_charger"
            elif r.leg == "charging" and r.battery_pct >= CHARGE_COMPLETE_THRESHOLD:
                r.leg = "idle"
                r.goal = self._nearest_parking(r.position, blocked)
            elif r.leg not in ("charging", "to_charger", "debug"):
                pk = self._nearest_parking(r.position, blocked)
                if r.position != pk:
                    r.goal = pk
                    r.leg = "to_parking"

    def _free_charger(self, pos: Coord, blocked: set[Coord]) -> Coord | None:
        chargers = cells_of_type(self.grid, CellType.CHARGER)
        used = {r.goal for r in self.robots if r.leg in ("charging", "to_charger")}
        dist = self.grid.bfs_distances(pos)
        best = None
        best_d = 10**6
        for ch in chargers:
            if ch in used:
                continue
            d = dist.get(ch, 10**6)
            if d < best_d:
                best_d = d
                best = ch
        return best

    def _free_pick_station(self) -> Coord | None:
        stations = cells_of_type(self.grid, CellType.PICK_STATION)
        used = set()
        for o in self.orders:
            if o.pick_station and o.completed_tick is None:
                used.add(o.pick_station)
        for r in self.robots:
            if r.leg == "to_station" and r.goal:
                used.add(r.goal)
        for st in stations:
            if st not in used:
                return st
        return None

    def _replan(self) -> None:
        self._regoal_unreachable()
        self._update_idle_goals()
        agents, plan_blocked = self._planning_context()
        if not agents:
            return
        t0 = time.monotonic()
        result = plan_joint(
            self.grid,
            plan_blocked,
            agents,
            self.committed_paths if self.committed_paths else None,
            seed=self.seed + self.tick,
            time_budget_s=self._replan_time_budget_s(),
        )
        planning_ms = (time.monotonic() - t0) * 1000.0
        aligned: dict[str, list[Coord]] = {}
        for r in self.robots:
            path = result.paths.get(r.robot_id, [r.position])
            if path and path[0] == r.position:
                aligned[r.robot_id] = path
            elif r.position in path:
                aligned[r.robot_id] = path[path.index(r.position) :]
            else:
                aligned[r.robot_id] = [r.position]
        result.paths = aligned

        min_len = (self.tick - self.plan_start_tick) + 60
        padded: dict[str, list[Coord]] = {}
        for rid, path in result.paths.items():
            if not path:
                padded[rid] = path
                continue
            if len(path) < min_len:
                tail = path[-1]
                padded[rid] = path + [tail] * (min_len - len(path))
            else:
                padded[rid] = path
        for r in self.robots:
            if r.robot_id not in padded:
                padded[r.robot_id] = [r.position]
        self.committed_paths = padded
        self.plan_start_tick = self.tick
        self.planner_used = result.planner_used
        for r in self.robots:
            r.path_start_tick = self.tick
            r.path = result.paths.get(r.robot_id, [r.position])

        makespan = 0
        soc = 0
        for aid, (start, goal) in agents.items():
            p = result.paths.get(aid, [])
            if start != goal:
                off = path_soc(p, goal)
                soc += off
                makespan = max(makespan, off)

        self.metrics.record_plan(
            result.planner_used,
            planning_ms,
            result.cbs.conflicts_resolved,
            makespan,
            soc,
        )

    def _joint_next_positions(self) -> dict[str, Coord]:
        nxt: dict[str, Coord] = {}
        for r in self.robots:
            if r.status in ("blocked", "deadlocked") or r.dwell_remaining > 0:
                nxt[r.robot_id] = r.position
                continue
            rel = self.tick - self.plan_start_tick
            path = self.committed_paths.get(r.robot_id, [r.position])
            idx = min(rel + 1, len(path) - 1)
            nxt[r.robot_id] = path[idx] if path else r.position
        return nxt

    def _made_progress(self, robot: Robot, old_pos: Coord) -> bool:
        if robot.goal and robot.position == robot.goal:
            return True
        path = self.committed_paths.get(robot.robot_id, [])
        if path and old_pos in path:
            idx = path.index(old_pos)
            if robot.position == old_pos:
                return False
            if idx + 1 < len(path) and robot.position == path[idx + 1]:
                return True
            if robot.position in path[: idx + 2]:
                return True
        if robot.goal:
            dist = self.grid.bfs_distances(robot.goal)
            before = dist.get(old_pos, 10**6)
            after = dist.get(robot.position, 10**6)
            if after < before:
                return True
        return False

    def _validate_step(self, nxt: dict[str, Coord]) -> bool:
        blocked = set(self.blocked_cells)
        for r in self.robots:
            if r.battery_pct <= 0:
                blocked.add(r.position)
        for rid, c in nxt.items():
            if not self.grid.is_walkable(c) or c in blocked:
                return False
        cells = list(nxt.values())
        if len(cells) != len(set(cells)):
            return False
        for a in self.robots:
            ca = nxt[a.robot_id]
            pa = a.position
            if abs(ca[0] - pa[0]) + abs(ca[1] - pa[1]) > 1:
                return False
        for i, a in enumerate(self.robots):
            for b in self.robots[i + 1 :]:
                ca, cb = nxt[a.robot_id], nxt[b.robot_id]
                pa, pb = a.position, b.position
                if ca == pb and cb == pa and ca != cb:
                    return False
        return True

    def _check_stalls(self) -> None:
        for r in self.robots:
            if r.status in ("deadlocked", "blocked"):
                continue
            if (
                self.planner_used == "hold"
                and r.waited_this_tick
                and r.current_order_id
                and r.goal
                and r.goal != r.position
                and r.dwell_remaining == 0
            ):
                r.hold_streak += 1
            elif self.planner_used != "hold":
                r.hold_streak = 0

            stalled = False
            if r.hold_streak >= 15:
                stalled = True
            if (
                r.goal
                and r.goal != r.position
                and r.dwell_remaining == 0
                and r.status not in ("charging", "dwelling")
            ):
                if r.ticks_without_progress >= 15:
                    stalled = True

            if stalled:
                self.metrics.stalls += 1
                r.stall_attempts += 1
                if r.stall_attempts >= 3:
                    if r.status != "deadlocked":
                        self.metrics.deadlocks += 1
                        r.status = "deadlocked"
                        r.goal = r.position
                else:
                    blocked = set(self.blocked_cells)
                    r.goal = self._nearest_parking(r.position, blocked)
                    if r.current_order_id:
                        for o in self.orders:
                            if o.order_id == r.current_order_id:
                                o.assigned_robot_id = None
                                o.pick_station = None
                                o.picked_tick = None
                                break
                        r.current_order_id = None
                    r.ticks_without_progress = 0
                    r.hold_streak = 0
                self._replan_queued = True

    def step(self) -> dict:
        self._apply_commands()

        pending = [
            o for o in self.orders if o.assigned_robot_id is None and o.completed_tick is None
        ]
        if pending and any(
            r.current_order_id is None and r.status not in ("charging", "deadlocked", "blocked")
            for r in self.robots
        ):
            self._run_allocation()

        if self._replan_queued:
            self._replan()
            self._replan_queued = False

        nxt = self._joint_next_positions()
        if not self._validate_step(nxt):
            for r in self.robots:
                nxt[r.robot_id] = r.position
                r.waited_this_tick = True
            self._replan_queued = True
        else:
            for r in self.robots:
                r.waited_this_tick = False
                if (
                    r.leg == "to_shelf"
                    and r.current_order_id
                    and r.position == r.goal
                    and r.dwell_remaining == 0
                ):
                    self._on_dwell_complete(r)
                if r.dwell_remaining > 0:
                    r.dwell_remaining -= 1
                    if r.dwell_remaining == 0:
                        self._on_dwell_complete(r)
                    continue
                target = nxt[r.robot_id]
                old_pos = r.position
                moved = target != r.position
                r.position = target
                if moved:
                    r.battery_pct = apply_move_drain(r.battery_pct)
                    self.metrics.record_move()
                    if self._made_progress(r, old_pos):
                        r.ticks_without_progress = 0
                    else:
                        r.ticks_without_progress += 1
                else:
                    r.waited_this_tick = True
                    on_charger = self.grid.cell_types.get(r.position) is CellType.CHARGER
                    if r.status == "charging" or on_charger:
                        if r.leg in ("charging", "to_charger") and r.position == r.goal:
                            r.battery_pct = apply_charge(r.battery_pct)
                            r.status = "charging"
                            r.leg = "charging"
                    if r.goal and r.position == r.goal and r.dwell_remaining == 0:
                        pass
                    elif r.goal and r.goal != r.position:
                        r.ticks_without_progress += 1

                if r.battery_pct <= 0 and r.status != "blocked":
                    r.status = "blocked"
                    self._replan_queued = True

                if r.goal and r.position == r.goal:
                    self._on_reach_goal(r)

                if moved:
                    r.status = "moving"
                elif r.status == "moving":
                    r.status = "idle"

        self._check_stalls()

        if self.auto_orders:
            active = sum(1 for o in self.orders if o.completed_tick is None)
            if active < self.robot_count and self.tick % 4 == 0:
                shelf = self._free_shelf()
                if shelf:
                    self.orders.append(
                        Order(
                            order_id=self._next_order_id(),
                            shelf_cell=shelf,
                            created_tick=self.tick,
                        )
                    )
                    self._replan_queued = True

        self.tick += 1
        return self._tick_frame()

    def _on_reach_goal(self, r: Robot) -> None:
        if r.leg == "to_shelf" and r.current_order_id:
            r.status = "dwelling"
            r.dwell_remaining = 2
        elif r.leg == "to_station":
            r.status = "dwelling"
            r.dwell_remaining = 2
        elif r.leg == "to_charger":
            r.leg = "charging"
            r.status = "charging"
        elif r.leg in ("to_parking", "parking", "idle", "debug"):
            r.status = "idle"

    def _on_dwell_complete(self, r: Robot) -> None:
        if r.leg == "to_shelf" and r.current_order_id:
            st = self._free_pick_station()
            if st:
                r.goal = st
                r.leg = "to_station"
                r.status = "moving"
                for o in self.orders:
                    if o.order_id == r.current_order_id:
                        o.pick_station = st
                        o.picked_tick = self.tick
                self._replan_queued = True
            else:
                r.status = "idle"
                self._replan_queued = True
        elif r.leg == "to_station" and r.current_order_id:
            for o in self.orders:
                if o.order_id == r.current_order_id:
                    o.completed_tick = self.tick
                    self.metrics.record_completion(self.tick)
            r.current_order_id = None
            r.leg = "idle"
            blocked = set(self.blocked_cells)
            r.goal = self._nearest_parking(r.position, blocked)
            r.status = "idle"
            self._replan_queued = True

    def _tick_frame(self) -> dict:
        return {
            "type": "tick",
            "tick": self.tick,
            "robots": self._robots_payload(),
            "orders": self._orders_payload(),
            "blocked_cells": [[c[0], c[1]] for c in sorted(self.blocked_cells)],
            "planner_used": self.planner_used,
            "metrics": self.metrics.snapshot(self.tick),
        }

    def snapshot(self) -> dict:
        return {
            "type": "snapshot",
            "map": {
                "width": self.grid.width,
                "height": self.grid.height,
                "cells": self.map_rows,
            },
            "tick": self.tick,
            "robots": self._robots_payload(),
            "orders": self._orders_payload(),
            "blocked_cells": [[c[0], c[1]] for c in sorted(self.blocked_cells)],
            "metrics": self.metrics.snapshot(self.tick),
        }

    def _robots_payload(self) -> list[dict]:
        out = []
        for r in self.robots:
            out.append(
                {
                    "id": r.robot_id,
                    "pos": [r.position[0], r.position[1]],
                    "battery_pct": round(r.battery_pct, 1),
                    "status": r.status,
                    "order_id": r.current_order_id,
                }
            )
        return out

    def _orders_payload(self) -> list[dict]:
        out = []
        for o in self.orders:
            if o.completed_tick is not None:
                status = "completed"
            elif o.assigned_robot_id:
                status = "active"
            else:
                status = "pending"
            out.append(
                {
                    "id": o.order_id,
                    "shelf_cell": [o.shelf_cell[0], o.shelf_cell[1]],
                    "status": status,
                }
            )
        return out
