from __future__ import annotations

from app.models import CellType, Coord, Grid

DEFAULT_MAP: list[str] = [
    "########################",
    "#H#H#H#H#H#H#C#C#C#C#C##",
    "#......................#",
    "#.#a##a##a#..#a##a##a#.#",
    "#.##a##a###..##a##a###.#",
    "#......................#",
    "#.#a##a##a#..#a##a##a#.#",
    "#.##a##a###..##a##a###.#",
    "#......................#",
    "#.#a##a##a#..#a##a##a#.#",
    "#.##a##a###..##a##a###.#",
    "#......................#",
    "#P#P#P#P#H#H#H#H#H#H####",
    "########################",
]

_CHAR_TO_TYPE: dict[str, CellType] = {
    "#": CellType.WALL,
    ".": CellType.CORRIDOR,
    "a": CellType.SHELF_ACCESS,
    "P": CellType.PICK_STATION,
    "C": CellType.CHARGER,
    "H": CellType.PARKING,
}


def grid_from_rows(rows: list[str]) -> Grid:
    height = len(rows)
    width = len(rows[0]) if rows else 0
    cell_types: dict[Coord, CellType] = {}
    for r, row in enumerate(rows):
        for c, ch in enumerate(row):
            cell_types[(r, c)] = _CHAR_TO_TYPE.get(ch, CellType.WALL)
    return Grid(width=width, height=height, cell_types=cell_types)


def default_grid() -> Grid:
    return grid_from_rows(DEFAULT_MAP)


def map_rows_to_cell_strings(rows: list[str]) -> list[str]:
    return list(rows)


def cells_of_type(grid: Grid, cell_type: CellType) -> list[Coord]:
    return sorted([c for c, t in grid.cell_types.items() if t is cell_type])


def is_corridor(grid: Grid, c: Coord) -> bool:
    return grid.cell_types.get(c) is CellType.CORRIDOR


def is_endpoint_cell(grid: Grid, c: Coord) -> bool:
    ct = grid.cell_types.get(c)
    if ct is None or ct is CellType.WALL or ct is CellType.CORRIDOR:
        return False
    return True
