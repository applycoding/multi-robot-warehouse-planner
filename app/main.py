from __future__ import annotations

import asyncio
import os
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from app.grid import DEFAULT_MAP
from app.simulation import Command, SimulationEngine
from app.ws_protocol import (
    BlockCellRequest,
    ConfigRequest,
    ControlRequest,
    RobotCountRequest,
)

WEB_DIR = Path(__file__).resolve().parent.parent / "web"

TICK_SIM_SECONDS = 0.5
BASE_TICKS_PER_SEC = 4.0


class ClientState:
    def __init__(self, ws: WebSocket) -> None:
        self.ws = ws
        self.queue: asyncio.Queue[dict | None] = asyncio.Queue(maxsize=1)
        self.sender_task: asyncio.Task | None = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    seed = int(os.environ.get("SIM_SEED", "1"))
    autostart = os.environ.get("SIM_AUTOSTART", "1") != "0"
    engine = SimulationEngine(
        map_rows=DEFAULT_MAP,
        seed=seed,
        auto_orders=True,
    )
    engine.playing = autostart
    app.state.engine = engine
    app.state.clients: list[ClientState] = []
    app.state.tick_task = asyncio.create_task(_tick_loop(app))
    yield
    app.state.tick_task.cancel()
    try:
        await app.state.tick_task
    except asyncio.CancelledError:
        pass
    for client in app.state.clients:
        if client.sender_task:
            client.sender_task.cancel()


app = FastAPI(lifespan=lifespan)


async def _tick_loop(app: FastAPI) -> None:
    while True:
        engine: SimulationEngine = app.state.engine
        if engine.playing:
            interval = 1.0 / (BASE_TICKS_PER_SEC * engine.speed)
            await asyncio.sleep(interval)
            frame = await asyncio.to_thread(engine.step)
            await _broadcast(app, frame)
        else:
            await asyncio.to_thread(engine.process_pending_commands)
            await asyncio.sleep(0.1)


async def _broadcast(app: FastAPI, frame: dict) -> None:
    for client in app.state.clients:
        try:
            if client.queue.full():
                try:
                    client.queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            client.queue.put_nowait(frame)
        except asyncio.QueueFull:
            pass


async def _client_sender(client: ClientState) -> None:
    while True:
        frame = await client.queue.get()
        if frame is None:
            break
        await client.ws.send_json(frame)


@app.get("/healthz")
def healthz() -> dict:
    return {"status": "ok"}


@app.post("/api/orders")
def create_order() -> dict:
    app.state.engine.enqueue(Command("add_order"))
    return {"ok": True}


@app.post("/api/block-cell")
def block_cell(req: BlockCellRequest) -> dict:
    err = app.state.engine.try_block_cell_api(req.row, req.col, req.blocked)
    if err:
        raise HTTPException(status_code=400, detail=err)
    return {"ok": True}


@app.post("/api/robots/count")
def robot_count(req: RobotCountRequest) -> dict:
    if req.count < 4 or req.count > 12:
        raise HTTPException(status_code=400, detail="count must be 4-12")
    app.state.engine.enqueue(Command("robot_count", {"count": req.count}))
    return {"ok": True}


@app.post("/api/control")
def control(req: ControlRequest) -> dict:
    if req.speed is not None and (req.speed < 0.25 or req.speed > 8):
        raise HTTPException(status_code=400, detail="speed out of range")
    app.state.engine.enqueue(
        Command("control", {"action": req.action, "speed": req.speed})
    )
    return {"ok": True}


@app.post("/api/config")
def config(req: ConfigRequest) -> dict:
    payload = {}
    if req.allocator is not None:
        payload["allocator"] = req.allocator
    if req.seed is not None:
        payload["seed"] = req.seed
    if req.auto_orders is not None:
        payload["auto_orders"] = req.auto_orders
    app.state.engine.enqueue(Command("config", payload))
    return {"ok": True}


@app.post("/api/reset")
def reset() -> dict:
    app.state.engine.enqueue(Command("reset"))
    return {"ok": True}


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    engine: SimulationEngine = app.state.engine
    client = ClientState(ws)
    app.state.clients.append(client)
    await ws.send_json(engine.snapshot())
    client.sender_task = asyncio.create_task(_client_sender(client))
    try:
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        if client.sender_task:
            client.sender_task.cancel()
        app.state.clients.remove(client)


@app.get("/")
def index() -> FileResponse:
    return FileResponse(WEB_DIR / "index.html")


app.mount("/static", StaticFiles(directory=WEB_DIR), name="static")
