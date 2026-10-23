"""Regression: robots must not gain charge unless on a charger cell."""

from app.grid import CellType
from app.simulation import SimulationEngine


def test_no_charging_off_charger_cell() -> None:
    eng = SimulationEngine(robot_count=8, seed=1, auto_orders=True)
    off_charger_charge_ticks = 0
    for _ in range(400):
        eng.step()
        for r in eng.robots:
            on_charger = eng.grid.cell_types.get(r.position) is CellType.CHARGER
            if r.status == "charging" and not on_charger:
                off_charger_charge_ticks += 1
    assert off_charger_charge_ticks == 0
