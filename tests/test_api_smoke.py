import os
import socket
import subprocess
import sys
import time

import pytest
from playwright.sync_api import Page, expect


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def server_url() -> str:
    port = _free_port()
    env = os.environ.copy()
    env["SIM_SEED"] = "1"
    env["SIM_AUTOSTART"] = "0"
    env["E2E_SIMPLE_MAP"] = "1"
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
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    import httpx2

    url = f"http://127.0.0.1:{port}"
    for _ in range(50):
        try:
            r = httpx2.get(f"{url}/healthz", timeout=1.0)
            if r.status_code == 200:
                break
        except Exception:
            time.sleep(0.2)
    else:
        proc.kill()
        raise RuntimeError("server failed to start")
    yield url
    proc.terminate()
    proc.wait(timeout=10)


@pytest.mark.e2e
def test_ui_smoke(page: Page, server_url: str) -> None:
    page.goto(server_url)
    canvas = page.locator("canvas")
    expect(canvas).to_be_visible()
    page.wait_for_function("() => window.__lastFrame && window.__lastFrame.type === 'snapshot'")
    cell_px = page.evaluate("window.__cellPx")
    assert cell_px > 0
    page.locator("#pause-btn").click()
    block_row, block_col = 2, 3
    page.locator("canvas").click(
        position={
            "x": block_col * cell_px + cell_px // 2,
            "y": block_row * cell_px + cell_px // 2,
        }
    )
    page.locator("#play-btn").click()
    page.wait_for_function(
        """() => {
          const f = window.__lastFrame;
          return f && f.blocked_cells && f.blocked_cells.length > 0;
        }"""
    )
    tick_before = page.locator("[data-testid='tick-counter']").inner_text()
    page.locator("#pause-btn").click()
    page.locator("#play-btn").click()
    expect(page.locator("[data-testid='tick-counter']")).not_to_have_text(tick_before)
