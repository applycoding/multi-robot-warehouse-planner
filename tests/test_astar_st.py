from app.grid import grid_from_rows
from app.planner.astar_st import ConstraintTable, space_time_astar


def test_manhattan_path_length() -> None:
    grid = grid_from_rows([".....", ".....", "....."])
    path = space_time_astar(
        grid, "a", (0, 0), (2, 4), 0, ConstraintTable(), horizon=50
    )
    assert path is not None
    assert len(path) == 2 + 4 + 1


def test_vertex_constraint_respected() -> None:
    grid = grid_from_rows(["....."])
    c = ConstraintTable()
    c.forbid_vertex("a", (0, 2), 2)
    path = space_time_astar(
        grid, "a", (0, 0), (0, 4), 0, c, horizon=50
    )
    assert path is not None
    assert path[2] != (0, 2)


def test_edge_constraint_respected() -> None:
    grid = grid_from_rows(["....."])
    c = ConstraintTable()
    c.forbid_edge("a", (0, 1), (0, 2), 2)
    path = space_time_astar(
        grid, "a", (0, 0), (0, 4), 0, c, horizon=50
    )
    assert path is not None
    for k in range(1, len(path)):
        if path[k] == (0, 2) and path[k - 1] == (0, 1):
            assert k != 2


def test_goal_latest_constraint() -> None:
    grid = grid_from_rows(["....."])
    c = ConstraintTable()
    c.forbid_vertex("a", (0, 4), 3)
    path = space_time_astar(
        grid, "a", (0, 0), (0, 4), 0, c, horizon=50
    )
    assert path is not None
    arrival = next(i for i, cell in enumerate(path) if cell == (0, 4))
    assert arrival > c.latest_goal_constraint("a", (0, 4))


def test_horizon_returns_none() -> None:
    grid = grid_from_rows(["....."])
    c = ConstraintTable()
    for k in range(1, 6):
        c.forbid_vertex("a", (0, 4), k)
    path = space_time_astar(grid, "a", (0, 0), (0, 4), 0, c, horizon=3)
    assert path is None
