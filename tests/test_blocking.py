"""Regression: default-map cell blocking connectivity check."""

from app.grid import DEFAULT_MAP
from app.simulation import SimulationEngine


def test_block_ordinary_aisle_on_default_map_succeeds() -> None:
    eng = SimulationEngine(map_rows=DEFAULT_MAP, robot_count=4, seed=1, auto_orders=False)
    err = eng.try_block_cell_api(5, 10, True)
    assert err is None
    eng.process_pending_commands()
    assert (5, 10) in eng.blocked_cells


def test_block_articulation_cell_on_default_map_rejected() -> None:
    eng = SimulationEngine(map_rows=DEFAULT_MAP, robot_count=4, seed=1, auto_orders=False)
    err = eng.try_block_cell_api(2, 3, True)
    assert err == "disconnect"
