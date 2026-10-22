#!/usr/bin/env python3
from __future__ import annotations

import os
import socket
import subprocess
import sys
import time
from pathlib import Path

from playwright.sync_api import sync_playwright


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def main() -> None:
    root = Path(__file__).resolve().parent.parent
    out = root / "docs" / "screenshot.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    port = _free_port()
    env = os.environ.copy()
    env["SIM_SEED"] = "42"
    env["SIM_AUTOSTART"] = "1"
    proc = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "uvicorn",
            "app.main:app",
            "--host",
            "127.0.0.1",
            "--port",
            str(port),
        ],
        cwd=root,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    import httpx2

    url = f"http://127.0.0.1:{port}"
    try:
        for _ in range(60):
            try:
                if httpx2.get(f"{url}/healthz", timeout=1.0).status_code == 200:
                    break
            except Exception:
                time.sleep(0.25)
        else:
            raise RuntimeError("server did not start")

        httpx2.post(f"{url}/api/robots/count", json={"count": 8}, timeout=5.0)
        httpx2.post(
            f"{url}/api/config",
            json={"allocator": "greedy", "auto_orders": True},
            timeout=5.0,
        )
        for _ in range(3):
            httpx2.post(f"{url}/api/orders", json={}, timeout=5.0)
        httpx2.post(
            f"{url}/api/block-cell",
            json={"row": 2, "col": 8, "blocked": True},
            timeout=5.0,
        )
        httpx2.post(f"{url}/api/control", json={"action": "play", "speed": 4.0}, timeout=5.0)

        with sync_playwright() as p:
            browser = p.chromium.launch()
            page = browser.new_page(viewport={"width": 1280, "height": 800})
            page.goto(url, wait_until="networkidle")
            page.wait_for_timeout(6000)
            page.screenshot(path=str(out), full_page=True)
            browser.close()
    finally:
        proc.terminate()
        proc.wait(timeout=10)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
