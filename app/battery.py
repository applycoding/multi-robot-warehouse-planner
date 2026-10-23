from __future__ import annotations

DRAIN_PER_MOVE = 0.5
CHARGE_PER_TICK = 5.0
LOW_BATTERY_THRESHOLD = 40.0
CHARGE_COMPLETE_THRESHOLD = 90.0


def apply_move_drain(battery_pct: float) -> float:
    return max(0.0, battery_pct - DRAIN_PER_MOVE)


def apply_charge(battery_pct: float) -> float:
    return min(100.0, battery_pct + CHARGE_PER_TICK)


def allocator_battery_reserve(
    dist_to_shelf: int,
    dist_shelf_to_station: int,
    dist_station_to_charger: int,
) -> float:
    total_moves = dist_to_shelf + dist_shelf_to_station + dist_station_to_charger
    return DRAIN_PER_MOVE * total_moves + 10.0
