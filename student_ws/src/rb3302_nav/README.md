# Programmed inputs and path preview

Use the provided `planning_astar` package for map loading, EKF, AMCL and navigation.
There is no separate planning-only mode or launch in this package.

## Files

- `config/run.yaml`: starting square, bearing and four ordered goals (repeats allowed).
- `../../maps/course_letters.yaml`: letter coordinates, map file and heading offset.
- `rb3302_nav/course.py`: validate inputs and convert coordinates.
- `rb3302_nav/path_preview.py`: request the programmed start-to-first-goal path from
  the running Nav2 planner. Formerly stage1_runner.py.
- `rb3302_nav/render.py`: save the returned path as PNG and text.
- `test/test_stage1.py`: offline input, coordinate and rendering tests.
- setup/package/resource files: install this ROS Python package.

The preview uses the programmed start, not the AMCL estimate, and does not initialise
AMCL or command motion. The normal navigation costmap still requires working robot
transforms and localisation. Four-goal mission execution is not implemented yet.
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
ROS execution must be verified on the lab machine; this Mac has no ROS runtime.
