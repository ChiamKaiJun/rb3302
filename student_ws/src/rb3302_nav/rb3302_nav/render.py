"""Draw the loaded map, spawn pose, four goals, and the first A* path."""

from __future__ import annotations

import math

import numpy as np
from PIL import Image, ImageDraw

from rb3302_nav.course import CourseMap


def path_directions(path: np.ndarray, heading_offset: float = 0.0) -> list[tuple[str, float]]:
    """Group path segments into eight course-compass directions for debugging."""
    names = ('E', 'NE', 'N', 'NW', 'W', 'SW', 'S', 'SE')
    groups = []
    for dx, dy in np.diff(np.asarray(path, dtype=float).reshape(-1, 2), axis=0):
        distance = math.hypot(dx, dy)
        if distance < 1e-9:
            continue
        angle = math.atan2(dy, dx) - heading_offset
        name = names[int(math.floor(angle / (math.pi / 4) + .5)) % 8]
        if groups and groups[-1][0] == name:
            groups[-1] = (name, groups[-1][1] + distance)
        else:
            groups.append((name, distance))
    return groups


def format_path_report(course, run, path: np.ndarray) -> str:
    """Describe the returned Nav2 route; includes raw map coordinates for replay."""
    x, y, yaw = course.start_pose(run)
    groups = path_directions(path, course.heading_offset)
    lines = [
        f'Nav2 A* first path: {run.start} -> {run.goals[0]}',
        f'Start: {run.start}, bearing {run.bearing}, map pose ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f} deg)',
        f'Ordered goals: {", ".join(run.goals)} (only goal 1 planned)',
        f'Goal 1 map position: {course.letters[run.goals[0]]}',
        f'Map: {course.map.yaml_path}',
        f'Letter coordinates calibrated: {course.calibrated}; robot radius: {course.robot_radius:.3f} m',
        f'Path: {len(path)} points, {sum(length for _, length in groups):.3f} m',
        'Directions: ' + (' -> '.join(f'{name} {length:.2f} m' for name, length in groups) or 'no translation'),
        'Directions are rounded to 8 course-compass headings, not movement instructions.',
        'Raw path points (index, map x metres, map y metres):',
    ]
    lines.extend(f'{i:04d}: {px:.6f}, {py:.6f}' for i, (px, py) in enumerate(path))
    return '\n'.join(lines) + '\n'


def render_stage1(
    course: CourseMap,
    spawn_xy: tuple[float, float],
    yaw: float,
    goals: list[tuple[str, tuple[float, float]]],
    path: np.ndarray,
    out_path: str,
    scale: int = 6,
    landmarks: dict | None = None,
) -> None:
    width = course.nx * scale
    height = course.ny * scale
    header, footer = 32, 42
    # Occupancy is row-major with +y north; image row 0 is the top of the picture.
    occupied = np.flipud(course.occupied)
    unknown = np.flipud(course.unknown)
    img = Image.new("RGB", (width, height + header + footer), (245, 245, 245))
    pix = img.load()
    cell = scale
    for j in range(course.ny):
        for i in range(course.nx):
            color = (20, 20, 20) if occupied[j, i] else ((190, 190, 190) if unknown[j, i] else (255, 255, 255))
            for dy in range(cell):
                for dx in range(cell):
                    pix[i * cell + dx, j * cell + dy + header] = color
    draw = ImageDraw.Draw(img)
    draw.text((8, 8), 'First-goal path | blue: start heading | arrows: path direction', fill=(0, 0, 0))
    draw.text((8, header + height + 5), 'Red: path   Green: goal 1   Orange: later goals', fill=(0, 0, 0))
    draw.text((8, header + height + 22), 'Black: occupied   Grey: unknown   White: free', fill=(0, 0, 0))

    def to_px(x: float, y: float) -> tuple[float, float]:
        local = course.rotation.T @ (np.array([x, y]) - course.origin_xy)
        px = local[0] / course.resolution * cell
        py = (course.height_m - local[1]) / course.resolution * cell + header
        return px, py

    for letter, xy in (landmarks or {}).items():
        px, py = to_px(*xy)
        draw.text((px + 3, py + 3), letter, fill=(100, 100, 100))
    if len(path) >= 2:
        draw.line([to_px(x, y) for x, y in path], fill=(220, 40, 40), width=3)
        # Arrowheads every ~0.25 m; their screen direction accounts for map yaw.
        travelled = 0.0
        for start, end in zip(path[:-1], path[1:]):
            travelled += float(np.linalg.norm(end-start))
            if travelled < .25:
                continue
            ax, ay = to_px(*start)
            bx, by = to_px(*end)
            angle = math.atan2(by-ay, bx-ax)
            if math.hypot(bx-ax, by-ay) > 1e-9:
                c, s = math.cos(angle), math.sin(angle)
                draw.polygon([(bx, by), (bx-9*c+4*s, by-9*s-4*c),
                              (bx-9*c-4*s, by-9*s+4*c)], fill=(220, 40, 40))
                travelled = 0.0
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
        ty = max(header, min(py-16, header+height-16))
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
