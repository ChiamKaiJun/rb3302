"""Run Stage 1 objectives 1-3 and report whether each check passed."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from stage1_sim.course import CourseMap
from stage1_sim.grid import bearing_yaw, validate_run_inputs
from stage1_sim.planner import path_stays_free, plan_path
from stage1_sim.render import render_stage1


@dataclass
class GoalCheck:
    letter: str
    x: float
    y: float
    cell: str | None

    @property
    def ok(self) -> bool:
        return self.cell == self.letter


@dataclass
class Stage1Result:
    start: str
    bearing: str
    yaw: float
    spawn_xy: tuple[float, float]
    spawn_cell: str | None
    goals: list[GoalCheck]
    path: np.ndarray
    first_goal: str
    path_reaches_goal: bool
    path_free: bool
    notes: list[str] = field(default_factory=list)

    @property
    def spawn_ok(self) -> bool:
        return self.spawn_cell == self.start

    @property
    def goals_ok(self) -> bool:
        return all(goal.ok for goal in self.goals)

    @property
    def plan_ok(self) -> bool:
        return self.path_reaches_goal and self.path_free

    @property
    def ok(self) -> bool:
        return self.spawn_ok and self.goals_ok and self.plan_ok


def run_stage1(start: str, bearing: str, goals: list[str], resolution: float = 0.04) -> Stage1Result:
    start, bearing, goals = validate_run_inputs(start, bearing, goals)
    course = CourseMap(resolution=resolution)
    yaw = bearing_yaw(bearing)
    spawn = course.cells[start].center
    checks = []
    for letter in goals:
        x, y = course.cells[letter].center
        checks.append(GoalCheck(letter, x, y, course.letter_at(x, y)))
    path = plan_path(course, spawn, (checks[0].x, checks[0].y))
    reaches = _path_reaches(course, path, checks[0].letter)
    result = Stage1Result(
        start=start,
        bearing=bearing,
        yaw=yaw,
        spawn_xy=spawn,
        spawn_cell=course.letter_at(*spawn),
        goals=checks,
        path=path,
        first_goal=checks[0].letter,
        path_reaches_goal=reaches,
        path_free=path_stays_free(course, path),
    )
    result._course = course  # used by the renderer; not part of the public checks
    return result


def save_preview(result: Stage1Result, out_path: str) -> None:
    course = result._course
    render_stage1(
        course,
        result.spawn_xy,
        result.yaw,
        [(goal.letter, (goal.x, goal.y)) for goal in result.goals],
        result.path,
        out_path,
    )


def format_report(result: Stage1Result) -> str:
    lines = [
        f"spawn {result.start} {result.bearing}: "
        f"x={result.spawn_xy[0]:.3f} y={result.spawn_xy[1]:.3f} "
        f"cell={result.spawn_cell} {'OK' if result.spawn_ok else 'FAIL'}",
    ]
    for index, goal in enumerate(result.goals, start=1):
        mark = "OK" if goal.ok else "FAIL"
        role = "  <- A* target" if index == 1 else ""
        lines.append(
            f"goal {index} {goal.letter}: x={goal.x:.3f} y={goal.y:.3f} cell={goal.cell} {mark}{role}"
        )
    lines.append(
        f"A* {result.start} -> {result.first_goal}: "
        f"{len(result.path)} waypoints, free={result.path_free}, "
        f"reaches={result.path_reaches_goal} {'OK' if result.plan_ok else 'FAIL'}"
    )
    lines.append(f"stage 1.1-1.3 {'PASS' if result.ok else 'FAIL'}")
    return "\n".join(lines)


def _path_reaches(course: CourseMap, path: np.ndarray, letter: str) -> bool:
    if len(path) == 0:
        return False
    return course.letter_at(float(path[-1, 0]), float(path[-1, 1])) == letter
