from __future__ import annotations

from collections import deque


class MetricsCollector:
    def __init__(self) -> None:
        self.throughput_total = 0
        self.completion_ticks: deque[int] = deque(maxlen=120)
        self.cumulative_moves = 0
        self.conflicts_resolved = 0
        self.planning_times: deque[float] = deque(maxlen=100)
        self.planning_time_last_ms = 0.0
        self.replans = 0
        self.planner_used_counts: dict[str, int] = {"cbs": 0, "pp": 0, "hold": 0}
        self.stalls = 0
        self.deadlocks = 0
        self.last_makespan = 0
        self.last_soc = 0

    def record_completion(self, tick: int) -> None:
        self.throughput_total += 1
        self.completion_ticks.append(tick)

    def record_move(self) -> None:
        self.cumulative_moves += 1

    def record_plan(
        self,
        planner_used: str,
        planning_ms: float,
        conflicts_resolved: int,
        last_makespan: int,
        last_soc: int,
    ) -> None:
        self.replans += 1
        self.planner_used_counts[planner_used] = self.planner_used_counts.get(planner_used, 0) + 1
        self.planning_time_last_ms = planning_ms
        self.planning_times.append(planning_ms)
        self.conflicts_resolved += conflicts_resolved
        self.last_makespan = last_makespan
        self.last_soc = last_soc

    def throughput_per_min(self, current_tick: int) -> float:
        if not self.completion_ticks:
            return 0.0
        window_ticks = min(120, current_tick) if current_tick > 0 else 1
        window_start = current_tick - window_ticks
        count = sum(1 for t in self.completion_ticks if t > window_start)
        if count == 0 and self.throughput_total > 0:
            count = self.throughput_total
            window_ticks = max(current_tick, 1)
        return count * (120.0 / window_ticks) * 2.0

    def planning_time_p95_ms(self) -> float:
        if not self.planning_times:
            return 0.0
        sorted_times = sorted(self.planning_times)
        idx = int(0.95 * (len(sorted_times) - 1))
        return sorted_times[idx]

    def snapshot(self, current_tick: int) -> dict:
        return {
            "throughput_per_min": round(self.throughput_per_min(current_tick), 2),
            "throughput_total": self.throughput_total,
            "last_makespan": self.last_makespan,
            "last_soc": self.last_soc,
            "cumulative_moves": self.cumulative_moves,
            "conflicts_resolved": self.conflicts_resolved,
            "planning_time_last_ms": round(self.planning_time_last_ms, 2),
            "planning_time_p95_ms": round(self.planning_time_p95_ms(), 2),
            "replans": self.replans,
            "planner_used_counts": dict(self.planner_used_counts),
            "stalls": self.stalls,
            "deadlocks": self.deadlocks,
        }

    def reset(self) -> None:
        self.__init__()
