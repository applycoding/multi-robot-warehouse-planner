const canvas = document.getElementById("grid");
const ctx = canvas.getContext("2d");
const tickVal = document.getElementById("tick-val");
const metricsEl = document.getElementById("metrics");
const errorEl = document.getElementById("error");

let mapData = null;
let cellPx = 24;
window.__lastFrame = null;
window.__cellPx = cellPx;

const colors = {
  wall: "#3d4f66",
  corridor: "#1e2a3a",
  shelf_access: "#6b8f71",
  pick_station: "#c9a227",
  charger: "#4fc3f7",
  parking: "#8e7cc3",
  blocked: "#b71c1c",
  robot: "#ff7043",
};

const charToType = {
  "#": "wall",
  ".": "corridor",
  a: "shelf_access",
  P: "pick_station",
  C: "charger",
  H: "parking",
};

function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === "snapshot") {
      mapData = msg.map;
      fitCanvas();
    }
    if (msg.type === "snapshot" || msg.type === "tick") {
      window.__lastFrame = msg;
      tickVal.textContent = String(msg.tick);
      render(msg);
      updateMetrics(msg.metrics || {});
    }
  };
  ws.onclose = () => setTimeout(connect, 1000);
}

function fitCanvas() {
  if (!mapData) return;
  const maxW = 960;
  const maxH = 560;
  cellPx = Math.floor(Math.min(maxW / mapData.width, maxH / mapData.height));
  cellPx = Math.max(cellPx, 12);
  window.__cellPx = cellPx;
  canvas.width = mapData.width * cellPx;
  canvas.height = mapData.height * cellPx;
}

function render(frame) {
  if (!mapData) return;
  const blocked = new Set((frame.blocked_cells || []).map((c) => `${c[0]},${c[1]}`));
  for (let r = 0; r < mapData.height; r++) {
    const row = mapData.cells[r];
    for (let c = 0; c < mapData.width; c++) {
      const ch = row[c];
      const t = charToType[ch] || "wall";
      ctx.fillStyle = colors[t] || colors.wall;
      ctx.fillRect(c * cellPx, r * cellPx, cellPx, cellPx);
      if (blocked.has(`${r},${c}`)) {
        ctx.fillStyle = colors.blocked;
        ctx.globalAlpha = 0.65;
        ctx.fillRect(c * cellPx, r * cellPx, cellPx, cellPx);
        ctx.globalAlpha = 1;
      }
    }
  }
  for (const robot of frame.robots || []) {
    const [r, c] = robot.pos;
    ctx.fillStyle = colors.robot;
    ctx.beginPath();
    ctx.arc(c * cellPx + cellPx / 2, r * cellPx + cellPx / 2, cellPx * 0.35, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "#fff";
    ctx.font = `${Math.max(10, cellPx * 0.35)}px sans-serif`;
    ctx.textAlign = "center";
    ctx.fillText(robot.id.replace("r", ""), c * cellPx + cellPx / 2, r * cellPx + cellPx / 2 + 4);
  }
}

function updateMetrics(m) {
  const keys = [
    "throughput_per_min",
    "throughput_per_min_cumulative",
    "last_makespan",
    "last_soc",
    "conflicts_resolved",
    "planning_time_last_ms",
    "replans",
    "planner_used_counts",
    "stalls",
    "deadlocks",
    "orders_cancelled_total",
  ];
  metricsEl.innerHTML = keys
    .map((k) => {
      const v = m[k] !== undefined ? JSON.stringify(m[k]) : "-";
      return `<li><strong>${k}</strong>: ${v}</li>`;
    })
    .join("");
}

async function post(path, body) {
  const res = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body ?? {}),
  });
  if (!res.ok) {
    const data = await res.json().catch(() => ({}));
    showError(data.detail || "Request failed");
    return false;
  }
  hideError();
  return true;
}

function showError(msg) {
  errorEl.textContent = msg;
  errorEl.hidden = false;
}

function hideError() {
  errorEl.hidden = true;
}

canvas.addEventListener("click", async (ev) => {
  const rect = canvas.getBoundingClientRect();
  const c = Math.floor((ev.clientX - rect.left) / cellPx);
  const r = Math.floor((ev.clientY - rect.top) / cellPx);
  const frame = window.__lastFrame;
  const blocked = new Set((frame?.blocked_cells || []).map((x) => `${x[0]},${x[1]}`));
  const isBlocked = blocked.has(`${r},${c}`);
  await post("/api/block-cell", { row: r, col: c, blocked: !isBlocked });
});

document.getElementById("play-btn").addEventListener("click", () => post("/api/control", { action: "play" }));
document.getElementById("pause-btn").addEventListener("click", () => post("/api/control", { action: "pause" }));
document.getElementById("add-order").addEventListener("click", () => post("/api/orders", {}));
document.getElementById("reset-btn").addEventListener("click", () => post("/api/reset", {}));
document.getElementById("speed").addEventListener("change", (e) => {
  post("/api/control", { action: "play", speed: parseFloat(e.target.value) });
});
document.getElementById("robot-count").addEventListener("change", (e) => {
  post("/api/robots/count", { count: parseInt(e.target.value, 10) });
});
document.getElementById("allocator").addEventListener("change", (e) => {
  post("/api/config", { allocator: e.target.value });
});

connect();
