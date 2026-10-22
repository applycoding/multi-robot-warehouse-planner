from app.allocator import greedy_assign, optimal_assign
from app.grid import grid_from_rows
from app.models import Order, Robot


def _cost_sum(assignments: dict[str, str], orders: list[Order], robots: list[Robot], grid) -> int:
    total = 0
    for oid, rid in assignments.items():
        order = next(o for o in orders if o.order_id == oid)
        robot = next(r for r in robots if r.robot_id == rid)
        d = grid.bfs_distances(order.shelf_cell)
        total += d.get(robot.position, 10**6)
    return total


def test_optimal_not_worse_than_greedy_and_strictly_better_once() -> None:
    grid = grid_from_rows(
        [
            "########",
            "#H.....#",
            "#......#",
            "#a.....#",
            "#a.....#",
            "########",
        ]
    )
    robots = [
        Robot("r1", (1, 1), 100.0, "idle", None),
        Robot("r2", (1, 6), 100.0, "idle", None),
    ]
    orders = [
        Order("o1", (3, 1), 1),
        Order("o2", (4, 6), 2),
    ]
    g = greedy_assign(orders, robots, grid)
    o = optimal_assign(orders, robots, grid)
    assert _cost_sum(o, orders, robots, grid) <= _cost_sum(g, orders, robots, grid)

    row = "." * 20
    grid2 = grid_from_rows(
        [
            "#" + row + "#",
            "#H" + "." * 18 + "H#",
            "#H" + "." * 16 + "PC#",
            "#" + row + "#",
        ]
    )
    robots2 = [
        Robot("r1", (1, 1), 100.0, "idle", None),
        Robot("r2", (1, 10), 100.0, "idle", None),
        Robot("r3", (1, 19), 100.0, "idle", None),
    ]
    orders2 = [
        Order("o1", (2, 2), 1),
        Order("o2", (2, 15), 2),
        Order("o3", (2, 16), 3),
    ]
    g2 = greedy_assign(orders2, robots2, grid2)
    o2 = optimal_assign(orders2, robots2, grid2)
    assert _cost_sum(o2, orders2, robots2, grid2) < _cost_sum(g2, orders2, robots2, grid2)


def test_only_oldest_r_orders() -> None:
    grid = grid_from_rows(["###", "#.#", "#.#", "###"])
    robots = [Robot("r1", (1, 1), 100.0, "idle", None)]
    orders = [
        Order("o1", (2, 1), 1),
        Order("o2", (2, 1), 2),
        Order("o3", (2, 1), 3),
    ]
    g = greedy_assign(orders, robots, grid)
    assert "o3" not in g
    assert "o1" in g or len(g) <= 1


def test_ineligible_not_assigned() -> None:
    grid = grid_from_rows(["#######", "#a.H.P#", "#C....#", "#######"])
    robots = [
        Robot("r1", (1, 1), 5.0, "idle", None),
        Robot("r2", (1, 3), 100.0, "idle", None),
    ]
    orders = [Order("o1", (1, 1), 1)]
    o = optimal_assign(orders, robots, grid)
    assert o.get("o1") == "r2"


def test_one_to_one() -> None:
    grid = grid_from_rows(["#####", "#...#", "#####"])
    robots = [
        Robot("r1", (1, 1), 100.0, "idle", None),
        Robot("r2", (1, 3), 100.0, "idle", None),
    ]
    orders = [Order("o1", (1, 2), 1), Order("o2", (1, 2), 2)]
    g = greedy_assign(orders, robots, grid)
    assert len(g) <= 2
    assert len(set(g.values())) == len(g)
