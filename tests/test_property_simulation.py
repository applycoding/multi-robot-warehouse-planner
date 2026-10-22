from hypothesis import given, settings
from hypothesis import strategies as st

from app.simulation import Command, SimulationEngine


@settings(max_examples=30, deadline=None, derandomize=True, database=None)
@given(
    seed=st.integers(0, 5000),
    robot_count=st.integers(4, 12),
    n_cmds=st.integers(0, 8),
)
def test_simulation_trajectory_invariants(seed: int, robot_count: int, n_cmds: int) -> None:
    eng = SimulationEngine(robot_count=robot_count, seed=seed, auto_orders=False)
    for i in range(n_cmds):
        if i % 3 == 0:
            eng.enqueue(Command("add_order"))
        elif i % 3 == 1:
            eng.enqueue(Command("block_cell", {"row": 2, "col": 3, "blocked": True}))
        else:
            eng.enqueue(Command("block_cell", {"row": 2, "col": 3, "blocked": False}))
    history: dict[int, dict[str, tuple[int, int]]] = {}
    for t in range(150):
        frame = eng.step()
        history[t] = {rb["id"]: tuple(rb["pos"]) for rb in frame["robots"]}
    for t in range(1, 150):
        prev = history[t - 1]
        cur = history[t]
        cells = list(cur.values())
        assert len(cells) == len(set(cells))
        for rid, pos in cur.items():
            assert eng.grid.is_walkable(pos)
            assert tuple(pos) not in {(c[0], c[1]) for c in eng.blocked_cells}
        for rid in cur:
            p0, p1 = prev[rid], cur[rid]
            assert abs(p0[0] - p1[0]) + abs(p0[1] - p1[1]) <= 1
        ids = sorted(cur.keys())
        for i, a in enumerate(ids):
            for b in ids[i + 1 :]:
                if cur[a] == prev[b] and cur[b] == prev[a] and cur[a] != cur[b]:
                    raise AssertionError("swap")
