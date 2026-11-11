"""Record one physically verified letter centre without commanding robot motion."""
import argparse
import math
import os
from pathlib import Path
import shutil
import tempfile
import time

import yaml

from rb3302_nav.course import Course, LETTERS


def mapping_value(node, name):
    if not isinstance(node, yaml.MappingNode):
        raise ValueError(f'Expected a YAML mapping containing {name}')
    matches = [value for key, value in node.value if key.value == name]
    if len(matches) != 1:
        raise ValueError(f'Expected exactly one YAML field named {name}')
    return matches[0]


def save_centre(course_file, letter, x, y):
    """Validate a centre and replace only its X/Y scalars, preserving YAML comments."""
    path = Path(course_file).expanduser().resolve()
    original = path.read_text()
    course = Course(path)
    if letter not in course.letters:
        raise ValueError(f'Missing letter {letter}')
    if not all(math.isfinite(value) for value in (x, y)):
        raise ValueError('Coordinates must be finite')
    i, j = course.map.world_to_grid(x, y)
    if course.map.occupancy[j, i]:
        raise ValueError('Measured centre is occupied or unknown in the saved map; check localisation')

    root = yaml.compose(original)
    letters = mapping_value(root, 'letters')
    point = mapping_value(letters, letter)
    edits = []
    for axis, value in [('x', x), ('y', y)]:
        scalar = mapping_value(point, axis)
        if not isinstance(scalar, yaml.ScalarNode) or not (
                point.start_mark.index <= scalar.start_mark.index < point.end_mark.index):
            raise ValueError('Letter coordinates must be explicit YAML scalars, without aliases')
        edits.append((scalar.start_mark.index, scalar.end_mark.index, f'{value:.6f}'))
    updated = original
    for start, end, replacement in sorted(edits, reverse=True):
        updated = updated[:start] + replacement + updated[end:]

    # Keep a previous version and replace atomically, so an interrupted write cannot
    # leave a partially written course file. The overall calibrated flag is unchanged.
    backup = path.with_name(path.name + '.bak')
    if path.read_text() != original:
        raise ValueError('Course file changed during calibration; retry')
    shutil.copy2(path, backup)
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        try:
            stream.write(updated)
            stream.flush()
            os.fsync(stream.fileno())
            temporary.chmod(path.stat().st_mode & 0o777)
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)
    return course.letters[letter], backup


def capture_position(timeout):
    """Read a fresh map -> base_link transform from the running localisation stack."""
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init(args=[])
    node = Node('calibrate_goal')
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    deadline = time.monotonic() + timeout
    reason = 'No map -> base_link transform received'
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                transform = buffer.lookup_transform('map', 'base_link', Time())
            except TransformException as error:
                reason = str(error)
                continue
            stamp = transform.header.stamp
            age = (node.get_clock().now().nanoseconds -
                   (stamp.sec * 1_000_000_000 + stamp.nanosec)) / 1e9
            if not -1.0 <= age <= 1.0:
                reason = f'Transform timestamp differs from current time by {age:.2f} seconds'
                continue
            position = transform.transform.translation
            return position.x, position.y
        raise RuntimeError(f'Could not read a fresh robot position: {reason}')
    finally:
        listener.unregister()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--letter', required=True, type=str.upper, choices=LETTERS)
    parser.add_argument('--course', required=True, type=Path,
                        help='Exact course YAML also passed to run_course')
    parser.add_argument('--x', type=float, help='Previously measured map X in metres')
    parser.add_argument('--y', type=float, help='Previously measured map Y in metres')
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='Seconds to wait for live TF (default: 10)')
    args = parser.parse_args(argv)
    if (args.x is None) != (args.y is None):
        parser.error('Supply both --x and --y, or omit both to capture live TF')
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    try:
        course = Course(args.course)
        if args.letter not in course.letters:
            raise ValueError(f'Missing letter {args.letter}')
        if args.x is None:
            print('Keep base_link stationary over the physical centre, with navigation '
                  'cancelled and localisation running. Reading map -> base_link...', flush=True)
            x, y = capture_position(args.timeout)
        else:
            x, y = args.x, args.y
        previous, backup = save_centre(args.course, args.letter, x, y)
    except (ValueError, OSError, RuntimeError, yaml.YAMLError) as error:
        parser.exit(1, f'Calibration failed: {error}\n')
    print(f'Saved {args.letter}: ({previous[0]:.6f}, {previous[1]:.6f}) -> '
          f'({x:.6f}, {y:.6f}) in map\nCourse: {args.course.expanduser().resolve()}\n'
          f'Previous version: {backup}\nOverall calibrated flag unchanged; '
          'rerun run_course to load this centre.')


if __name__ == '__main__':
    main()
