"""Shared map and landmark coordinates for offline and ROS checks.

Adapted from the supplied stage1_sim/course.py. Letter coordinates are explicit:
they are never guessed from the scanned free-space bounds.
"""
from dataclasses import dataclass
from pathlib import Path
import math

import numpy as np
from PIL import Image
import yaml

LETTERS = 'ABCDEFGHIJKLMNOPQRSTUVWX'
BEARINGS = {'E': 0, 'NE': 45, 'N': 90, 'NW': 135,
            'W': 180, 'SW': -135, 'S': -90, 'SE': -45}


def letter(value):
    result = str(value).strip().upper()
    if len(result) != 1 or result not in LETTERS:
        raise ValueError(f'Expected one letter A-X, got {value!r}')
    return result


@dataclass(frozen=True)
class Run:
    start: str
    bearing: str
    goals: tuple

    @property
    def yaw(self):
        return math.radians(BEARINGS[self.bearing])

    @classmethod
    def load(cls, path):
        data = yaml.safe_load(Path(path).read_text())
        if not isinstance(data, dict):
            raise ValueError('Run file must contain start, bearing, and goals')
        start = letter(data.get('start', ''))
        bearing = str(data.get('bearing', '')).strip().upper()
        if bearing not in BEARINGS:
            raise ValueError('Bearing must be N, NE, E, SE, S, SW, W, or NW')
        goals = data.get('goals')
        if not isinstance(goals, list) or len(goals) != 4:
            raise ValueError('Supply exactly four goal letters in an ordered YAML list')
        return cls(start, bearing, tuple(letter(g) for g in goals))


class CourseMap:
    def __init__(self, yaml_path):
        self.yaml_path = Path(yaml_path).expanduser().resolve()
        meta = yaml.safe_load(self.yaml_path.read_text())
        if meta.get('mode', 'trinary') != 'trinary':
            raise ValueError('Stage 1 supports trinary maps only')
        self.resolution = float(meta['resolution'])
        origin = np.asarray(meta['origin'], dtype=float)
        if origin.shape != (3,) or not np.isfinite(origin).all():
            raise ValueError('Map origin must contain three finite numbers')
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError('Map resolution must be positive')
        self.origin_xy, self.yaw = origin[:2], float(origin[2])
        self.rotation = np.array([[math.cos(self.yaw), -math.sin(self.yaw)],
                                  [math.sin(self.yaw), math.cos(self.yaw)]])
        with Image.open(self.yaml_path.parent / meta['image']) as image:
            self.grid = np.flipud(np.asarray(image.convert('L'))).copy()
        self.ny, self.nx = self.grid.shape
        self.height_m, self.width_m = self.ny*self.resolution, self.nx*self.resolution
        free, occupied = float(meta['free_thresh']), float(meta['occupied_thresh'])
        if not 0 <= free < occupied <= 1:
            raise ValueError('Require 0 <= free_thresh < occupied_thresh <= 1')
        negate = int(meta.get('negate', 0))
        if negate not in (0, 1):
            raise ValueError('negate must be 0 or 1')
        probability = self.grid.astype(float) / 255.0
        if not negate:
            probability = 1.0 - probability
        self.occupied = probability > occupied
        self.unknown = ~(self.occupied | (probability < free))
        self.occupancy = self.occupied | self.unknown

    def world_to_grid(self, x, y):
        local = self.rotation.T @ (np.array([x, y]) - self.origin_xy)
        i, j = np.floor(local / self.resolution).astype(int)
        if not (0 <= i < self.nx and 0 <= j < self.ny):
            raise ValueError(f'Point ({x:.3f}, {y:.3f}) is outside the map')
        return int(i), int(j)

    def grid_to_world(self, i, j):
        point = self.origin_xy + self.rotation @ (np.array([i+.5, j+.5])*self.resolution)
        return tuple(float(v) for v in point)



class Course:
    def __init__(self, path):
        self.path = Path(path).expanduser().resolve()
        data = yaml.safe_load(self.path.read_text())
        if data.get('frame_id') != 'map':
            raise ValueError('Course coordinates must use frame_id: map')
        self.calibrated = data.get('calibrated') is True
        self.map = CourseMap(self.path.parent / data['map_yaml'])
        self.heading_offset = math.radians(float(data.get('heading_offset_deg', 0)))
        self.robot_radius = float(data.get('robot_radius_m', .22))
        if not math.isfinite(self.heading_offset) or not math.isfinite(self.robot_radius) or self.robot_radius <= 0:
            raise ValueError('Heading must be finite and robot radius positive')
        self.letters = {}
        for key, point in data.get('letters', {}).items():
            key = letter(key)
            xy = (float(point['x']), float(point['y']))
            if not all(math.isfinite(v) for v in xy):
                raise ValueError(f'{key} has non-finite coordinates')
            self.map.world_to_grid(*xy)
            self.letters[key] = xy

    def validate(self, run):
        blocked = self.map.occupancy
        for key in (run.start, *run.goals):
            if key not in self.letters:
                raise ValueError(f'Missing measured coordinates for {key}')
            i, j = self.map.world_to_grid(*self.letters[key])
            if blocked[j, i]:
                raise ValueError(f'{key} is occupied or unknown; check its coordinates')

    def start_pose(self, run):
        return (*self.letters[run.start], run.yaw + self.heading_offset)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Check the programmed inputs without ROS')
    parser.add_argument('--run', required=True)
    parser.add_argument('--course', help='Also check letter coordinates against the map')
    args = parser.parse_args()
    try:
        run = Run.load(args.run)
        print(f'Start: {run.start}, bearing: {run.bearing}, goals: {list(run.goals)}')
        if args.course:
            course = Course(args.course)
            course.validate(run)
            print(f'Start in map: {course.start_pose(run)}')
            print(f'First goal in map: {course.letters[run.goals[0]]}')
    except (ValueError, KeyError, OSError) as error:
        parser.exit(1, f'{error}\n')
