from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel


class BlockCellRequest(BaseModel):
    row: int
    col: int
    blocked: bool


class RobotCountRequest(BaseModel):
    count: int


class ControlRequest(BaseModel):
    action: Literal["play", "pause"]
    speed: float | None = None


class ConfigRequest(BaseModel):
    allocator: Literal["greedy", "optimal"] | None = None
    seed: int | None = None
    auto_orders: bool | None = None


class OrderResponse(BaseModel):
    id: str
    shelf_cell: list[int]
    status: str


class RobotFrame(BaseModel):
    id: str
    pos: list[int]
    battery_pct: float
    status: str
    order_id: str | None = None


class TickFrame(BaseModel):
    type: Literal["tick"] = "tick"
    tick: int
    robots: list[RobotFrame]
    orders: list[OrderResponse]
    blocked_cells: list[list[int]]
    planner_used: str
    metrics: dict[str, Any]


class SnapshotFrame(BaseModel):
    type: Literal["snapshot"] = "snapshot"
    map: dict[str, Any]
    tick: int
    robots: list[RobotFrame]
    orders: list[OrderResponse]
    blocked_cells: list[list[int]]
    metrics: dict[str, Any]
