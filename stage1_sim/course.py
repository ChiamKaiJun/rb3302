"""Load the saved course map and place A-X with simulation grid coordinates.

Real clicked coordinates are not available yet. The 4x6 letter grid is fitted
to the free-space rectangle of student_ws/maps/rb3302_map.pgm. A-F is the
north row. S is the south-west cell.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from stage1_sim.grid import LETTERS, letter_col_row


@dataclass(frozen=True)
class Cell:
    letter: str
    col: int
    row: int
    x0: float
    y0: float
    size_x: float
    size_y: float
    center_xy: tuple[float, float] | None = None

    @property
    def center(self) -> tuple[float, float]:
        if self.center_xy is not None:
            return self.center_xy
        return self.x0 + 0.5 * self.size_x, self.y0 + 0.5 * self.size_y

    def contains(self, x: float, y: float) -> bool:
        return self.x0 <= x < self.x0 + self.size_x and self.y0 <= y < self.y0 + self.size_y


def default_map_yaml() -> Path:
    return Path(__file__).resolve().parents[1] / "student_ws" / "maps" / "rb3302_map.yaml"


class CourseMap:
    def __init__(self, yaml_path: str | Path | None = None) -> None:
        path = Path(yaml_path) if yaml_path else default_map_yaml()
        meta = _read_map_yaml(path)
        image = _read_pgm(path.parent / meta["image"])
        self.resolution = meta["resolution"]
        self.origin_xy = np.array(meta["origin"][:2], dtype=float)
        self.image_height, self.image_width = image.shape
        # Store south at row 0 so world_to_grid matches +y north.
        self.grid = np.flipud(image)
        self.occupancy = self.grid != 254
        self.ny, self.nx = self.occupancy.shape
        self.width_m = self.nx * self.resolution
        self.height_m = self.ny * self.resolution
        self.cells = self._fit_letters()

    def _fit_letters(self) -> dict[str, Cell]:
        free = np.flipud(self.grid == 254)
        rows, cols = np.where(free)
        if len(rows) == 0:
            raise ValueError("Saved map has no free cells")
        y0, y1 = int(rows.min()), int(rows.max())
        x0, x1 = int(cols.min()), int(cols.max())
        pitch_x = (x1 - x0 + 1) * self.resolution / 6.0
        pitch_y = (y1 - y0 + 1) * self.resolution / 4.0
        south_west_x = self.origin_xy[0] + x0 * self.resolution
        south_west_y = self.origin_xy[1] + y0 * self.resolution
        cells = {}
        for letter in LETTERS:
            col, row = letter_col_row(letter)
            cell = Cell(
                letter,
                col,
                row,
                south_west_x + col * pitch_x,
                south_west_y + row * pitch_y,
                pitch_x,
                pitch_y,
            )
            cells[letter] = self._snap_center_free(cell)
        return cells

    def _snap_center_free(self, cell: Cell) -> Cell:
        """Keep the cell rectangle, but put its recorded centre on a free pixel."""
        cx, cy = cell.center
        i, j = self.world_to_grid(cx, cy)
        if not self.occupancy[j, i]:
            return cell
        best = None
        best_d = 1e9
        j0 = int(np.floor((cell.y0 - self.origin_xy[1]) / self.resolution))
        j1 = int(np.ceil((cell.y0 + cell.size_y - self.origin_xy[1]) / self.resolution))
        i0 = int(np.floor((cell.x0 - self.origin_xy[0]) / self.resolution))
        i1 = int(np.ceil((cell.x0 + cell.size_x - self.origin_xy[0]) / self.resolution))
        for jj in range(max(0, j0), min(self.ny, j1)):
            for ii in range(max(0, i0), min(self.nx, i1)):
                if self.occupancy[jj, ii]:
                    continue
                x, y = self.grid_to_world(ii, jj)
                if not cell.contains(x, y):
                    continue
                d = (x - cx) ** 2 + (y - cy) ** 2
                if d < best_d:
                    best_d = d
                    best = (x, y)
        if best is None:
            return cell
        return Cell(cell.letter, cell.col, cell.row, cell.x0, cell.y0, cell.size_x, cell.size_y, best)

    def letter_at(self, x: float, y: float) -> str | None:
        for letter, cell in self.cells.items():
            if cell.contains(x, y):
                return letter
        return None

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        i = int(np.floor((x - self.origin_xy[0]) / self.resolution))
        j = int(np.floor((y - self.origin_xy[1]) / self.resolution))
        return int(np.clip(i, 0, self.nx - 1)), int(np.clip(j, 0, self.ny - 1))

    def grid_to_world(self, i: int, j: int) -> tuple[float, float]:
        return (
            self.origin_xy[0] + (i + 0.5) * self.resolution,
            self.origin_xy[1] + (j + 0.5) * self.resolution,
        )

    def inflate(self, radius_m: float) -> np.ndarray:
        radius_cells = max(1, int(np.ceil(radius_m / self.resolution)))
        out = self.occupancy.copy()
        ys, xs = np.where(self.occupancy)
        for y, x in zip(ys, xs):
            y0 = max(0, int(y) - radius_cells)
            y1 = min(self.ny, int(y) + radius_cells + 1)
            x0 = max(0, int(x) - radius_cells)
            x1 = min(self.nx, int(x) + radius_cells + 1)
            out[y0:y1, x0:x1] = True
        return out


def _read_map_yaml(path: Path) -> dict:
    image = None
    resolution = None
    origin = None
    for raw in path.read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        key, value = key.strip(), value.strip()
        if key == "image":
            image = value
        elif key == "resolution":
            resolution = float(value)
        elif key == "origin":
            origin = [float(part) for part in value.strip("[]").split(",")]
    if not image or resolution is None or origin is None:
        raise ValueError(f"Map yaml {path} needs image, resolution and origin")
    return {"image": image, "resolution": resolution, "origin": origin}


def _read_pgm(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if not data.startswith(b"P5"):
        raise ValueError(f"{path} is not a binary PGM")
    idx = 0
    parts: list[bytes] = []
    while len(parts) < 4:
        end = data.find(b"\n", idx)
        line = data[idx:end]
        idx = end + 1
        if line.startswith(b"#"):
            continue
        parts.extend(line.split())
    width, height = int(parts[1]), int(parts[2])
    return np.frombuffer(data[idx : idx + width * height], dtype=np.uint8).reshape(height, width).copy()
