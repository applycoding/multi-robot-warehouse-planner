"""Long-run fleet: must keep completing orders (no charging freeze)."""

from app.grid import CellType
from app.simulation import SimulationEngine

TICKS = 800
BUCKET = 100


def _completion_buckets(eng: SimulationEngine) -> list[int]:
    buckets = [0] * (TICKS // BUCKET)
    for o in eng.orders:
        if o.completed_tick is None or o.picked_tick is None:
            continue
        idx = min((o.completed_tick - 1) // BUCKET, len(buckets) - 1)
        if o.completed_tick > 0:
            buckets[idx] += 1
    return buckets


def _run_long(seed: int, robot_count: int) -> dict:
    eng = SimulationEngine(robot_count=robot_count, seed=seed, auto_orders=True)
    collisions = 0
    full_charge_charging_streak: dict[str, int] = {}
    max_full_charge_streak = 0
    off_charger_charging = 0
    for _ in range(TICKS):
        frame = eng.step()
        pos = [tuple(rb["pos"]) for rb in frame["robots"]]
        if len(pos) != len(set(pos)):
            collisions += 1
        for r in eng.robots:
            on_charger = eng.grid.cell_types.get(r.position) is CellType.CHARGER
            if r.status == "charging" and not on_charger:
                off_charger_charging += 1
            if r.status == "charging" and r.battery_pct >= 90.0:
                full_charge_charging_streak[r.robot_id] = (
                    full_charge_charging_streak.get(r.robot_id, 0) + 1
                )
                max_full_charge_streak = max(
                    max_full_charge_streak,
                    full_charge_charging_streak[r.robot_id],
                )
            else:
                full_charge_charging_streak[r.robot_id] = 0
    buckets = _completion_buckets(eng)
    m = eng.metrics.snapshot(eng.tick)
    return {
        "buckets": buckets,
        "collisions": collisions,
        "deadlocks": m["deadlocks"],
        "stalls": m["stalls"],
        "off_charger_charging": off_charger_charging,
        "max_full_charge_streak": max_full_charge_streak,
    }


def test_fleet_completes_every_hundred_ticks_after_warmup() -> None:
    for robot_count in (4, 8, 12):
        for seed in (1, 2, 3):
            r = _run_long(seed, robot_count)
            assert r["collisions"] == 0
            assert r["off_charger_charging"] == 0
            assert r["max_full_charge_streak"] <= 5
            # buckets 1..7 are ticks 100-799 (skip first 100-tick warmup)
            for count in r["buckets"][1:]:
                assert count >= 1
            assert r["deadlocks"] <= robot_count * 8
