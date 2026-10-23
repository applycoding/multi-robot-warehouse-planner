from fastapi.testclient import TestClient

from app.main import app


def test_api_raise_robot_count_distinct_positions() -> None:
    with TestClient(app) as client:
        client.post("/api/reset")
        client.post("/api/config", json={"seed": 1, "auto_orders": False})
        client.post("/api/robots/count", json={"count": 4})
        eng = client.app.state.engine
        eng.process_pending_commands()
        eng.step()
        client.post("/api/robots/count", json={"count": 8})
        eng.process_pending_commands()
        eng.step()
        positions = [r.position for r in eng.robots]
    assert len(positions) == 8
    assert len(positions) == len(set(positions))
