"""4x6 landmark course used by the Stage 1 simulator.

Geometry matches the lab squares: 0.60 m floor, 0.08 m walls, ~0.46 m doors.
A-F is north. This is a static map only; localisation and driving are later stages.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from stage1_sim.grid import LETTERS, letter_col_row


@dataclass(frozen=True)
class Rect:
    x: float
    y: float
    w: float
    h: float


@dataclass(frozen=True)
class Cell:
    letter: str
    col: int
    row: int
    x0: float
    y0: float
    size: float

    @property
    def center(self) -> tuple[float, float]:
        return self.x0 + 0.5 * self.size, self.y0 + 0.5 * self.size

    def contains(self, x: float, y: float) -> bool:
        return self.x0 <= x <= self.x0 + self.size and self.y0 <= y <= self.y0 + self.size


class CourseMap:
    def __init__(self, resolution: float = 0.04) -> None:
        self.resolution = resolution
        self.cell_size = 0.60
        self.wall_t = 0.08
        self.margin = 0.10
        self.door_w = 0.46
        self.ncols, self.nrows = 6, 4
        self.pitch = self.cell_size + self.wall_t
        self.width_m = 2 * self.margin + self.ncols * self.cell_size + (self.ncols + 1) * self.wall_t
        self.height_m = 2 * self.margin + self.nrows * self.cell_size + (self.nrows + 1) * self.wall_t
        self.nx = int(round(self.width_m / resolution))
        self.ny = int(round(self.height_m / resolution))
        self.cells = {letter: self._make_cell(letter) for letter in LETTERS}
        self.occupancy = np.zeros((self.ny, self.nx), dtype=bool)
        self._rasterize(self._walls())

    def _origin(self, col: int, row: int) -> tuple[float, float]:
        return (
            self.margin + self.wall_t + col * self.pitch,
            self.margin + self.wall_t + row * self.pitch,
        )

    def _make_cell(self, letter: str) -> Cell:
        col, row = letter_col_row(letter)
        x0, y0 = self._origin(col, row)
        return Cell(letter, col, row, x0, y0, self.cell_size)

    def _doors(self) -> set[frozenset[str]]:
        doors: set[frozenset[str]] = set()
        for letter in LETTERS:
            col, row = letter_col_row(letter)
            if col < 5:
                right = LETTERS[(3 - row) * 6 + col + 1]
                doors.add(frozenset((letter, right)))
            if row < 3:
                north = LETTERS[(3 - (row + 1)) * 6 + col]
                doors.add(frozenset((letter, north)))
        blocked = {
            frozenset(("C", "D")),
            frozenset(("H", "I")),
            frozenset(("P", "Q")),
            frozenset(("V", "W")),
            frozenset(("B", "H")),
            frozenset(("E", "K")),
            frozenset(("O", "U")),
            frozenset(("Q", "W")),
            frozenset(("N", "O")),
        }
        return doors - blocked

    def _walls(self) -> list[Rect]:
        t = self.wall_t
        rects = [
            Rect(0.0, 0.0, self.width_m, t),
            Rect(0.0, self.height_m - t, self.width_m, t),
            Rect(0.0, 0.0, t, self.height_m),
            Rect(self.width_m - t, 0.0, t, self.height_m),
        ]
        doors = self._doors()

        def open_between(a: str, b: str) -> bool:
            return frozenset((a, b)) in doors

        for row in range(self.nrows):
            for col in range(self.ncols - 1):
                left = LETTERS[(3 - row) * 6 + col]
                right = LETTERS[(3 - row) * 6 + col + 1]
                x = self.margin + (col + 1) * self.pitch
                y0 = self.margin + row * self.pitch
                y1 = y0 + t + self.cell_size
                if open_between(left, right):
                    mid = 0.5 * (y0 + t + y1)
                    gap = 0.5 * self.door_w
                    rects.append(Rect(x, y0, t, max(0.02, mid - gap - y0)))
                    rects.append(Rect(x, mid + gap, t, max(0.02, y1 - (mid + gap))))
                else:
                    rects.append(Rect(x, y0, t, y1 - y0 + t))
        for row in range(self.nrows - 1):
            for col in range(self.ncols):
                south = LETTERS[(3 - row) * 6 + col]
                north = LETTERS[(3 - (row + 1)) * 6 + col]
                y = self.margin + (row + 1) * self.pitch
                x0 = self.margin + col * self.pitch
                x1 = x0 + t + self.cell_size
                if open_between(south, north):
                    mid = 0.5 * (x0 + t + x1)
                    gap = 0.5 * self.door_w
                    rects.append(Rect(x0, y, max(0.02, mid - gap - x0), t))
                    rects.append(Rect(mid + gap, y, max(0.02, x1 - (mid + gap)), t))
                else:
                    rects.append(Rect(x0, y, x1 - x0 + t, t))
        return rects

    def _rasterize(self, rects: list[Rect]) -> None:
        xs = (np.arange(self.nx) + 0.5) * self.resolution
        ys = (np.arange(self.ny) + 0.5) * self.resolution
        xx, yy = np.meshgrid(xs, ys)
        for rect in rects:
            self.occupancy |= (
                (xx >= rect.x) & (xx <= rect.x + rect.w) & (yy >= rect.y) & (yy <= rect.y + rect.h)
            )

    def letter_at(self, x: float, y: float) -> str | None:
        for letter, cell in self.cells.items():
            if cell.contains(x, y):
                return letter
        return None

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        i = int(np.floor(x / self.resolution))
        j = int(np.floor(y / self.resolution))
        return int(np.clip(i, 0, self.nx - 1)), int(np.clip(j, 0, self.ny - 1))

    def grid_to_world(self, i: int, j: int) -> tuple[float, float]:
        return (i + 0.5) * self.resolution, (j + 0.5) * self.resolution

    def inflate(self, radius_m: float) -> np.ndarray:
        """Mark cells whose centre is within radius_m of a wall."""
        radius_cells = int(math_ceil(radius_m / self.resolution))
        src = self.occupancy
        out = src.copy()
        ys, xs = np.where(src)
        for y, x in zip(ys, xs):
            y0 = max(0, int(y) - radius_cells)
            y1 = min(self.ny, int(y) + radius_cells + 1)
            x0 = max(0, int(x) - radius_cells)
            x1 = min(self.nx, int(x) + radius_cells + 1)
            out[y0:y1, x0:x1] = True
        return out


def math_ceil(value: float) -> int:
    return int(np.ceil(value))
