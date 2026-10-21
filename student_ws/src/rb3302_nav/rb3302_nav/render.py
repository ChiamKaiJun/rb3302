"""Draw the loaded map, spawn pose, four goals, and the first A* path."""

from __future__ import annotations

import math

import numpy as np
from PIL import Image, ImageDraw

from rb3302_nav.course import CourseMap


def render_stage1(
    course: CourseMap,
    spawn_xy: tuple[float, float],
    yaw: float,
    goals: list[tuple[str, tuple[float, float]]],
    path: np.ndarray,
    out_path: str,
    scale: int = 6,
) -> None:
    width = course.nx * scale
    height = course.ny * scale
    # Occupancy is row-major with +y north; image row 0 is the top of the picture.
    occ = np.flipud(course.occupancy)
    img = Image.new("RGB", (width, height), (245, 245, 245))
    pix = img.load()
    cell = scale
    for j in range(course.ny):
        for i in range(course.nx):
            color = (20, 20, 20) if occ[j, i] else (255, 255, 255)
            for dy in range(cell):
                for dx in range(cell):
                    pix[i * cell + dx, j * cell + dy] = color
    draw = ImageDraw.Draw(img)

    def to_px(x: float, y: float) -> tuple[float, float]:
        local = course.rotation.T @ (np.array([x, y]) - course.origin_xy)
        px = local[0] / course.resolution * cell
        py = (course.height_m - local[1]) / course.resolution * cell
        return px, py

    if len(path) >= 2:
        draw.line([to_px(x, y) for x, y in path], fill=(220, 40, 40), width=3)
    # Group repeated landmarks so their sequence numbers are all visible.
    groups = {}
    for index, (letter, xy) in enumerate(goals, start=1):
        groups.setdefault((letter, tuple(xy)), []).append(index)
    for (letter, (x, y)), indices in groups.items():
        px, py = to_px(x, y)
        r = 7
        fill = (30, 140, 60) if 1 in indices else (230, 140, 20)
        draw.ellipse((px-r, py-r, px+r, py+r), fill=fill, outline=(0, 0, 0))
        label = f"{','.join(map(str, indices))}:{letter}"
        bounds = draw.textbbox((0, 0), label)
        tx = max(0, min(px+8, width-(bounds[2]-bounds[0])-2))
        ty = max(0, min(py-16, height-16))
        draw.text((tx, ty), label, fill=(0, 0, 0))
    sx, sy = to_px(*spawn_xy)
    _robot(draw, sx, sy, yaw - course.yaw)
    img.save(out_path)


def _robot(draw: ImageDraw.ImageDraw, x: float, y: float, yaw: float) -> None:
    length, width = 16.0, 11.0
    # Image +y is south, so a world heading rotates the other way.
    heading = -yaw
    c, s = math.cos(heading), math.sin(heading)
    nose = (x + c * length, y + s * length)
    left = (x - c * 8 + -s * width, y - s * 8 + c * width)
    right = (x - c * 8 - -s * width, y - s * 8 - c * width)
    draw.polygon([nose, left, right], fill=(40, 90, 200), outline=(0, 0, 0))
