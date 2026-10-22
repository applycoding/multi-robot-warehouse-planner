from pathlib import Path

TRACKED_ROOT = Path(__file__).resolve().parent.parent
SKIP = {".git"}


def _iter_files() -> list[Path]:
    out: list[Path] = []
    for path in TRACKED_ROOT.rglob("*"):
        if not path.is_file():
            continue
        if any(part in SKIP for part in path.parts):
            continue
        if path.suffix in {".png", ".pyc"}:
            continue
        out.append(path)
    return out


def test_no_em_or_en_dash_in_tracked_files() -> None:
    bad: list[str] = []
    for path in _iter_files():
        text = path.read_text(encoding="utf-8", errors="replace")
        if "\u2014" in text or "\u2013" in text:
            bad.append(str(path))
    assert not bad, f"em/en dash found in: {bad}"
