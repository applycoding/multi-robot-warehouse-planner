from __future__ import annotations

from scipy.optimize import linear_sum_assignment

from app.battery import allocator_battery_reserve
from app.grid import cells_of_type
from app.models import CellType, Coord, Grid, Order, Robot


def _available_robots(robots: list[Robot]) -> list[Robot]:
    out: list[Robot] = []
    for r in robots:
        if r.status in ("deadlocked", "blocked"):
            continue
        if r.current_order_id is not None:
            continue
        if r.status == "charging":
            continue
        out.append(r)
    return out


def _pending_orders(orders: list[Order]) -> list[Order]:
    return [o for o in orders if o.completed_tick is None and o.assigned_robot_id is None]


def _nearest_charger_dist(grid: Grid, from_cell: Coord) -> int:
    chargers = cells_of_type(grid, CellType.CHARGER)
    if not chargers:
        return 0
    best = 10**6
    for ch in chargers:
        d = grid.bfs_distances(ch)
        if from_cell in d:
            best = min(best, d[from_cell])
    return best if best < 10**6 else 0


def _robot_eligible(grid: Grid, robot: Robot, order: Order) -> bool:
    if robot.battery_pct <= 0:
        return False
    shelf = order.shelf_cell
    d_robot = grid.bfs_distances(shelf)
    if robot.position not in d_robot:
        return False
    d_shelf = grid.bfs_distances(shelf)
    stations = cells_of_type(grid, CellType.PICK_STATION)
    best_station = 10**6
    for st in stations:
        ds = d_shelf.get(st, 10**6)
        if ds < best_station:
            best_station = ds
    if best_station >= 10**6:
        return False
    min_charger = _nearest_charger_dist(grid, stations[0] if stations else shelf)
    for st in stations:
        dc = _nearest_charger_dist(grid, st)
        min_charger = min(min_charger, dc)
    need = allocator_battery_reserve(
        d_robot[robot.position],
        best_station,
        min_charger,
    )
    return robot.battery_pct >= need


def _cost(grid: Grid, robot: Robot, order: Order) -> int:
    if not _robot_eligible(grid, robot, order):
        return 10**6
    d = grid.bfs_distances(order.shelf_cell)
    return d.get(robot.position, 10**6)


def greedy_assign(
    orders: list[Order],
    robots: list[Robot],
    grid: Grid,
) -> dict[str, str]:
    avail = _available_robots(robots)
    pending = sorted(_pending_orders(orders), key=lambda o: o.created_tick)
    r_count = len(avail)
    if r_count == 0:
        return {}
    consider = pending[:r_count]
    result: dict[str, str] = {}
    used: set[str] = set()
    for order in consider:
        best: Robot | None = None
        best_d = 10**6
        for robot in avail:
            if robot.robot_id in used:
                continue
            c = _cost(grid, robot, order)
            if c < best_d:
                best_d = c
                best = robot
        if best is not None and best_d < 10**6:
            result[order.order_id] = best.robot_id
            used.add(best.robot_id)
    return result


def optimal_assign(
    orders: list[Order],
    robots: list[Robot],
    grid: Grid,
) -> dict[str, str]:
    avail = _available_robots(robots)
    pending = sorted(_pending_orders(orders), key=lambda o: o.created_tick)
    r_count = len(avail)
    if r_count == 0:
        return {}
    consider = pending[:r_count]
    n_o = len(consider)
    n_r = len(avail)
    if n_o == 0:
        return {}

    cost_matrix = []
    for order in consider:
        row = [_cost(grid, robot, order) for robot in avail]
        cost_matrix.append(row)

    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    result: dict[str, str] = {}
    for ri, ci in zip(row_ind, col_ind, strict=True):
        if ri >= n_o or ci >= n_r:
            continue
        order = consider[ri]
        robot = avail[ci]
        if cost_matrix[ri][ci] >= 10**6:
            continue
        result[order.order_id] = robot.robot_id
    return result
