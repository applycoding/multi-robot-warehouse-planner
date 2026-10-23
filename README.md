# Multi-Robot Warehouse Planner

[![CI](https://github.com/applycoding/multi-robot-warehouse-planner/actions/workflows/ci.yml/badge.svg)](https://github.com/applycoding/multi-robot-warehouse-planner/actions/workflows/ci.yml)

CPU-only simulation of a warehouse AMR fleet with multi-agent path finding (CBS + prioritized planning fallback), task allocation, battery management, and a live canvas UI.

![Warehouse planner screenshot](docs/screenshot.png)

**This is a simulation demo. It is not connected to real robots, a fleet manager, or any production warehouse system.**

## Features

- Fixed grid warehouse map with shelf pockets, aisles, chargers, pick stations, and parking
- CBS MAPF with space-time A* low-level search, prioritized planning fallback, and HOLD backstop
- Greedy FCFS and optimal linear assignment (`scipy.optimize.linear_sum_assignment`) allocators
- Battery drain, charging, and eligibility checks
- FastAPI REST + WebSocket streaming to a vanilla JS canvas UI
- Metrics panel, cell blocking, robot count and speed controls

## Known limitations

- CBS falls back to prioritized planning (and then HOLD) around 10 to 12 robots on the default map; this is expected, not a defect.
- Robots are single-cell points on a grid (no footprint or clearance).
- Blocking is user-triggered only (no sensor obstacles).

## Quickstart

```sh
make install
make run
```

Open [http://localhost:8000](http://localhost:8000).

## Architecture

```mermaid
flowchart LR
    subgraph Backend [Python backend]
        SIM[SimulationEngine<br/>tick loop]
        ALLOC[Order Allocator<br/>greedy / optimal]
        CBS[Planner chain<br/>CBS -> PP -> HOLD]
        GRID[Grid / WarehouseMap]
        BAT[Battery Model]
        METRICS[Metrics Collector]
        VALID[Plan Validator]
    end
    subgraph API [FastAPI app]
        WS[WebSocket endpoint]
        HTTP[REST endpoints]
    end
    subgraph Frontend [web/ vanilla JS]
        CANVAS[Canvas 2D renderer]
        CONTROLS[Controls]
        PANEL[Metrics panel]
    end

    SIM --> ALLOC
    SIM --> CBS
    SIM --> BAT
    SIM --> METRICS
    CBS --> VALID
    ALLOC --> GRID
    CBS --> GRID
    SIM --> WS
    WS --> CANVAS
    WS --> PANEL
    CONTROLS --> HTTP
    HTTP --> SIM
    WS -- state frames --> CONTROLS
```

## How it works

Each tick, the simulation may allocate pending orders, then runs a full joint replan through the planner chain: CBS (400 node cap, 45 to 65% of the replan budget), prioritized planning with several robot orderings (remaining budget), then HOLD if needed. Replans use an adaptive wall-clock budget of `min(1.5 s, 0.3 s + 0.1 s × (n − 4))` (for example `0.3 s` at 4 robots and `1.1 s` at 12, capped at `1.5 s`) so CBS can finish on busy maps; unit tests still use the fixed `0.3 s` default on small instances. Space-time A* respects vertex and edge constraints, including the goal-cell constraint fix for target conflicts, and stops at the first valid goal arrival. The engine executes one step of the committed plan and validates collisions before applying motion.

## Tests

```sh
make test      # unit + property (no browser)
make e2e       # Playwright smoke test
```

## Docker

```sh
make docker-build
make docker-run
```

## Open-RMF concept note (naming only)

This repo does not use ROS 2 or Open-RMF. Conceptual analogies only:

- Allocators resemble Open-RMF `BidNotice` / `BidProposal` task-dispatch bidding (`rmf_task_msgs`)
- Robot state fields resemble `rmf_fleet_msgs` `RobotState` / `FleetState`
- Charging routing resembles `rmf_fleet_msgs` `ChargingAssignment`
- A committed plan resembles `rmf_traffic` `Itinerary` / `Route`

## Deviations from PLAN.md

- `cbs_plan` defaults to `max_consecutive_wait=1` for the PLAN target-conflict unit test; the simulation planner chain and oracle regression call CBS with `max_consecutive_wait=None`.
- Crossing SOC unit test uses a 3x3 grid with hand-verified optimal SOC `6` (brute-force joint BFS on detour-rich paths is slower to align).
- CBS uses 45% of the planner budget for five or fewer moving agents and 65% for larger subsets; PP keeps the remainder. Simulation replan budgets scale with robot count (see How it works).
- Dwelling, charging, and idle parked robots are excluded from joint planning and treated as static obstacles for that tick.
- While paused, the tick loop applies queued REST commands (play/pause/block) without advancing simulation ticks so the UI can resume.
- CI docker health check uses `127.0.0.1` and a short startup wait before `curl`.

## License

MIT License. See [LICENSE](LICENSE).
