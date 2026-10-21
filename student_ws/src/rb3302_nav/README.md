# Stage 1: inputs -> first Nav2 path -> picture

Scope: accept S0, B0 and exactly four ordered goals; plan S0 to S1; save the actual
returned Nav2 path as a PNG. Repeated goals are preserved. No AMCL, synthetic
sensors, odometry, controller or robot movement is involved.

## Read these files in this order

| File | Job | Why keep it? |
| --- | --- | --- |
| `config/run.yaml` | Start letter, compass bearing, four goals | Inputs changed before each run |
| `../../maps/course_letters.yaml` | A-X coordinates and heading offset on the actual map | Letters have no meaning to Nav2 without x/y coordinates |
| `rb3302_nav/course.py` | Validate input, load map and convert coordinates | Prevent invalid letters, missing centres and image-axis mistakes |
| `config/stage1.yaml` | Configure existing Nav2 A* and static costmap | Reuses ROS algorithms; no custom planner |
| `launch/stage1.launch.py` | Start map server, planner and input client | One command for the check |
| `rb3302_nav/stage1_runner.py` | Send explicit start and first goal to Nav2 | `use_start=True` removes the need for localisation |
| `rb3302_nav/render.py` | Draw the actual returned path on the map | CA2-style visual checking after every successful request |

`setup.py`, `setup.cfg`, `package.xml`, `resource/rb3302_nav`, and `__init__.py` are
ROS Python package plumbing. `test/test_stage1.py` tests input/map handling and
rendering; it does not implement or test Nav2's A* algorithm.

## 1. Enter the run

Edit `config/run.yaml`:

```yaml
start: S
bearing: E
goals: [T, N, N, A]
```

Keep all three keys at the same indentation. Bearings: N, NE, E, SE, S, SW, W, NW. Letter case is normalised.
Exactly four goals are required; order and repeats are preserved.

From repository root, input validation can run without ROS if NumPy, Pillow and
PyYAML are installed in your Python environment:

```bash
PYTHONPATH=student_ws/src/rb3302_nav python3 -m rb3302_nav.course \
  --run student_ws/src/rb3302_nav/config/run.yaml
```

## 2. Define the letter centres

`student_ws/maps/course_letters.yaml` refers to the existing map YAML. Fill the
measured map coordinates for each distinct letter used in the run. For example,
the syntax is `S: {x: <measured x>, y: <measured y>}` under `letters:`. Do not paste
placeholder numbers. The file now contains estimated centres from the user-confirmed
`letter_grid_preview.png` overlay. They are not yet physically measured.

- Coordinates are metres in the ROS `map` frame.
- `heading_offset_deg` is the map angle of the course's EAST direction, positive
  counter-clockwise. Map yaw = compass bearing yaw + this offset.
- `calibrated` records whether you have verified the measurements; it does not
  block this planning-only experiment. It is printed with the result.
- `robot_radius_m: 0.22` is a provisional enclosing circle, not a physical measurement.
  Nav2 uses it for collision costs. Measure protrusions before movement testing.
- The user confirmed pixel-only edits to the map. Its original origin and resolution
  remain in `rb3302_map.yaml`; grayscale pixels are interpreted using its thresholds.

Add `--course student_ws/maps/course_letters.yaml` to the validation command to
check the coordinates. Missing, out-of-map, occupied or unknown centres fail clearly.
Nav2 checks path reachability and footprint clearance; we do not write another A*.

## 3. Ask Nav2 for the first path (ROS 2 Jazzy lab machine)

From `student_ws`:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/rb3302_nav --ignore-src -r -y
colcon build --symlink-install --packages-select rb3302_nav
source install/setup.bash
ros2 launch rb3302_nav stage1.launch.py \
  course_file:="$PWD/maps/course_letters.yaml" \
  run_file:="$PWD/src/rb3302_nav/config/run.yaml" \
  output_file:=/tmp/first_path.png
```

Run inputs are read at startup. The client requests a path once and exits after
saving it; the map/planner processes remain running until Ctrl+C. Change the inputs
and relaunch to test another programmed start. Each successful request writes the
chosen output image (overwriting an existing image at that path). A failed request
returns an error; an older PNG is not evidence of a successful new plan.

The image shows walls/unknown in dark colours, the programmed start as a blue
heading triangle, first goal in green, later goals in orange, and the returned path
in red. Repeated goal labels are grouped, e.g. `2,3:N`.

Nav2's costmap needs a robot-frame TF even with an explicit start. This launch supplies
`map -> planning_start`, representing only the programmed input. It does not publish
`map -> odom` or `odom -> base_link`, and does not claim to know the robot's real pose.
Do not run another Nav2 launch with the same node names concurrently.

## What was removed

The added AMCL/EKF setup, synthetic sensors and ray caster, demo maps, coordinate
recorder, saved RViz layout, and duplicate offline A* / preview command were removed.
The original provided `planning_astar`, `planning_rpp`, `mapping_cartographer`, and
`robot_ws` packages are unchanged. We can return to localisation after this check works.

CA2's useful idea here is inspecting a drawn path over the occupancy grid. Nav2
NavFn produces the route, so it need not match CA2's four-neighbour Manhattan A*.
The CA1 file inspected is reactive obstacle avoidance, outside this stage's scope.
Heading is accepted and displayed; NavFn's 2D global search does not model steering
constraints or guarantee that the first segment matches the starting heading.

## Tests and limits

```bash
PYTHONPATH=student_ws/src/rb3302_nav python3 -m unittest discover \
  -s student_ws/src/rb3302_nav/test -v
```

Tests cover all eight bearings, repeats/order, malformed inputs, map thresholds,
unknown space, rotated map origin, bounds, missing/blocked landmarks, and PNG output.
Actual Nav2 planning must be checked on ROS 2 Jazzy. It has not been executed on this
Mac. All 24 estimated letter centres were checked to lie in known free map cells.
Physical calibration and an actual Nav2 planning run are still pending.

## View only the occupancy map in RViz

On the ROS 2 Jazzy lab machine, terminal 1 (from `student_ws`):

```bash
source /opt/ros/jazzy/setup.bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$PWD/maps/rb3302_map.yaml"
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
rviz2
```

Set Global Options -> Fixed Frame to `map`. Add a Map display with Topic `/map`,
Durability `Transient Local`, and Reliability `Reliable`. RViz receives a
`nav_msgs/OccupancyGrid`; it does not open the PGM directly. No robot or AMCL is
needed to display this map. The A-X overlay is not included in `/map`.
If the Stage 1 launch already runs, its map server is active: just start RViz,
without launching a second map server or repeating its lifecycle transitions.
