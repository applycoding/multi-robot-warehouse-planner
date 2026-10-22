from app.grid import grid_from_rows
from app.planner.cbs import cbs_plan
from app.planner.validate import validate_joint_plan, validate_trajectory


def test_validate_good_cbs() -> None:
    grid = grid_from_rows(["#####", ".....", "##.##"])
    agents = {"A": ((2, 2), (1, 2)), "B": ((1, 0), (1, 4))}
    res = cbs_plan(grid, frozenset(), agents)
    assert res.paths is not None
    assert validate_joint_plan(grid, res.paths) == []


def test_validate_vertex_collision() -> None:
    grid = grid_from_rows(["....."])
    paths = {"A": [(0, 0), (0, 1), (0, 2)], "B": [(0, 2), (0, 1), (0, 0)]}
    violations = validate_joint_plan(grid, paths)
    assert violations


def test_validate_trajectory_swap() -> None:
    grid = grid_from_rows(["....."])
    positions = {
        0: {"A": (0, 0), "B": (0, 2)},
        1: {"A": (0, 2), "B": (0, 0)},
    }
    violations = validate_trajectory(grid, positions, {0: set(), 1: set()})
    assert violations
