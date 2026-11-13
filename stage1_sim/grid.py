"""Stage 1.1: manual grid input. Spawn is a letter plus a compass heading."""

from __future__ import annotations

import math

LETTERS = "ABCDEFGHIJKLMNOPQRSTUVWX"
BEARING_DEG = {
    "E": 0.0,
    "NE": 45.0,
    "N": 90.0,
    "NW": 135.0,
    "W": 180.0,
    "SW": -135.0,
    "S": -90.0,
    "SE": -45.0,
}


def normalise_letter(value: str) -> str:
    letter = str(value).strip().upper()
    if letter not in LETTERS:
        raise ValueError(f"Expected one letter A-X, got {value!r}")
    return letter


def normalise_bearing(value: str) -> str:
    bearing = str(value).strip().upper()
    if bearing not in BEARING_DEG:
        raise ValueError("Bearing must be N, NE, E, SE, S, SW, W or NW")
    return bearing


def validate_run_inputs(start: str, bearing: str, goals: list[str] | tuple[str, ...]) -> tuple[str, str, list[str]]:
    """Spawn letter, heading, and exactly four destination letters.

    Order and repeats are kept. Stage 1.3 plans only the first destination;
    the other three are checked so they land in the named squares.
    """
    start = normalise_letter(start)
    bearing = normalise_bearing(bearing)
    if not isinstance(goals, (list, tuple)) or len(goals) != 4:
        raise ValueError("Supply exactly four destination letters")
    return start, bearing, [normalise_letter(g) for g in goals]


def bearing_yaw(bearing: str) -> float:
    return math.radians(BEARING_DEG[normalise_bearing(bearing)])


def letter_col_row(letter: str) -> tuple[int, int]:
    """A-F is the north row. Row 0 is the south row S-X."""
    index = LETTERS.index(normalise_letter(letter))
    col = index % 6
    row = 3 - index // 6
    return col, row
