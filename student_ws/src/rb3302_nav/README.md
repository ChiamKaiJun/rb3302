# Programmed inputs and path preview

Use the provided `planning_astar` package for map loading, EKF, AMCL and navigation.
There is no separate planning-only mode or launch in this package.

## Files

- `config/run.yaml`: starting square, bearing and four ordered goals (repeats allowed).
- `../../maps/course_letters.yaml`: letter coordinates, map file and heading offset.
- `rb3302_nav/course.py`: validate inputs and convert coordinates.
- `rb3302_nav/calibrate_goal.py`: save a physically verified letter centre from live TF
  or previously measured map coordinates.
- `rb3302_nav/path_preview.py`: request the programmed start-to-first-goal path from
  the running Nav2 planner. Formerly stage1_runner.py.
- `rb3302_nav/render.py`: save the returned path as PNG and text.
- `test/test_stage1.py`: offline input, coordinate and rendering tests.
- setup/package/resource files: install this ROS Python package.

The preview uses the programmed start, not the AMCL estimate, and does not initialise
AMCL or command motion. The normal navigation costmap still requires working robot
transforms and localisation. `run_course` executes four ordered letter goals through
the running Nav2 waypoint follower.
The course robot radius metadata is no longer applied to Nav2 by a wrapper: actual
collision settings now come exclusively from planning_astar/config/nav2_params.yaml.
That provided configuration has a small original footprint; measure and correct it
before autonomous movement. Letter coordinates are image-derived, not calibrated.

## Build from the main project folder on the ROS machine

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to rb3302_nav
source install/setup.bash
```

`source` loads the environment into this terminal. `--packages-up-to` builds this
package and workspace dependencies, including planning_astar. Old installed launch
files may remain after rebuilding; do not run the removed stage1 launch or first_path
entry point. The supported commands are below.

## First get AMCL working

Start robot drivers on the robot if not already running:

```bash
ros2 launch robot_init robot.launch.py
```

From the main project folder in another sourced ROS terminal:

```bash
ros2 launch planning_astar planning.launch.py map:="$PWD/student_ws/maps/rb3302_map.yaml" params_file:="$PWD/student_ws/src/planning_astar/config/nav2_params.yaml"
```

`map` selects the map YAML; `params_file` selects the source configuration to tune.
`$PWD` means the current folder. The launch opens RViz. Set 2D Pose Estimate to the
robot's approximate actual pose. Teleop without sending a navigation goal:

```bash
ros2 run waverover_base keyboard_teleop
```

Compare laser alignment with several walls and measure pose error after a straight
move, 90-degree turn and return to the start. Check the estimated transform with:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

This prints robot position/orientation in the map frame. Ctrl+C stops the inspection.
Resolve missing transforms/scans before changing AMCL noise parameters.

## Calibrate letter centres

Calibration records where a physically verified centre is in the map; it does not
drive the robot, recognise a floor letter, or correct AMCL. First verify the lidar
mounting transform and scan-to-map alignment. Keep the same map loaded in Nav2 as
the map referenced by `course_letters.yaml`.

Build the new command once from `student_ws`, then source the workspace:

```bash
colcon build --symlink-install --packages-select rb3302_nav
source install/setup.bash
```

Cancel the active navigation mission, keep localisation running, and teleoperate
until `base_link` is stationary over the physical letter centre. From `student_ws`,
record F with:

```bash
ros2 run rb3302_nav calibrate_goal --letter F \
  --course "$PWD/maps/course_letters.yaml"
```

The command captures a fresh `map -> base_link` transform and writes only that
letter's X/Y. It preserves YAML comments, refuses occupied/unknown or out-of-map
points, and keeps the previous file as `course_letters.yaml.bak` (replaced on each
successful calibration). It times out after 10 seconds if fresh TF is unavailable.
Live capture uses the real-time clock of the hardware stack.

To save a previously measured position instead, without needing live ROS nodes:

```bash
ros2 run rb3302_nav calibrate_goal --letter F --x 2.396 --y 0.145 \
  --course "$PWD/maps/course_letters.yaml"
```

Repeat separately for each required letter; one measurement cannot determine the
other centres. The overall `calibrated` flag is not changed automatically. Set it
to `true` only after all centres and `heading_offset_deg` have been verified.
The flag controls a warning; it does not perform calibration. Record positions only
with reliable localisation; TF reports an estimate, not independent physical truth.

Rerun `run_course` with the same explicit `--course` path to use the updated goals.
No navigation restart or rebuild is needed after editing just the course YAML.
`--start F` also uses F's updated coordinates to initialise localisation, so start
physically at F with the specified bearing. Direct RViz goals do not read this YAML.

## Optional path preview after localisation works

Edit run.yaml, then in another sourced terminal from the main project folder:

```bash
ros2 run rb3302_nav path_preview --ros-args -p course_file:="$PWD/student_ws/maps/course_letters.yaml" -p run_file:="$PWD/student_ws/src/rb3302_nav/config/run.yaml" -p output_file:="$PWD/first_path.png"
```

`ros2 run` starts the client; `--ros-args -p` supplies ROS parameters. The first two
files define coordinates and inputs. The output is written on this computer after
a successful Nav2 request. A matching first_path.txt contains raw coordinates and
approximate compass directions for debugging. Failed requests leave any old output
untouched. The route need not follow letter centres or the selected starting heading.

## Validation

```bash
PYTHONPATH=student_ws/src/rb3302_nav python3 -m unittest discover -s student_ws/src/rb3302_nav/test -v
```

These offline tests need NumPy, Pillow and PyYAML. They do not test Nav2 startup.
Live navigation and physical calibration accuracy must be verified on the robot.
