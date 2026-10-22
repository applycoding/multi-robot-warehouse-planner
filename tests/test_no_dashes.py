import subprocess
from pathlib import Path

TRACKED_ROOT = Path(__file__).resolve().parent.parent


def _iter_files() -> list[Path]:
    out = subprocess.check_output(
        ["git", "ls-files"],
        cwd=TRACKED_ROOT,
        text=True,
    ).splitlines()
    paths: list[Path] = []
    for rel in out:
        path = TRACKED_ROOT / rel
        if not path.is_file():
            continue
        if path.suffix in {".png", ".pyc"}:
            continue
        paths.append(path)
    return paths


def test_no_em_or_en_dash_in_tracked_files() -> None:
    bad: list[str] = []
    for path in _iter_files():
        text = path.read_text(encoding="utf-8", errors="replace")
        if "\u2014" in text or "\u2013" in text:
            bad.append(str(path))
    assert not bad, f"em/en dash found in: {bad}"
