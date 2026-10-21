# RB3302: first-path check and localisation check

The `stage1.launch.py` check accepts S0, B0 and exactly four ordered goals; plan S0 to S1; save the actual
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

The image shows black occupied cells, grey unknown cells, A-X labels, the programmed
start as a blue heading triangle, first goal in green, later goals in orange, and
the returned path in red with direction arrowheads. Repeated goal labels are grouped,
e.g. `2,3:N`.

A matching `first_path.txt` is also written beside `first_path.png`. It contains:
start pose and bearing, all four goals, map/calibration information, total path length,
grouped 8-compass-direction segments, and all raw Nav2 path coordinates. The terminal
prints the summary. Segment directions are for debugging, not controller commands.
They use the course heading offset, whereas raw coordinates use the map frame.

Today's acceptance checks:

| Run | Expected result |
| --- | --- |
| S/E with [T,N,H,A] | S -> T path, correct heading marker and ordered goal labels |
| S/N with [T,N,H,A] | Starting marker faces N; NavFn may return the same 2D route |
| S/E with [T,N,N,A] | Duplicate N retained and labelled 2,3; still plan only S -> T |
| S/E with [S,N,H,A] | Same-cell first goal is handled; path length should be near zero |
| Longer route, e.g. S/E with [H,T,N,A] | Inspect wall clearance and direction changes |
| Invalid start/goal or three goals | Reject input before planning |

A no-path result must report a planner error, not draw a successful replacement route.
Changing heading alone need not change NavFn's route: its global search is 2D and
not heading-constrained. Inspect clearance in the Nav2 costmap; the PNG's path line
by itself does not show the entire swept robot outline.

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

## Localisation check using standard Nav2 AMCL

Two files: `launch/localization.launch.py` includes the installed Nav2
`nav2_bringup/launch/localization_launch.py`; `config/localization.yaml` supplies
our frame names and EKF sensor selections. Nav2 starts the map server, AMCL and
lifecycle manager. Our EKF supplies odom -> base_link. AMCL supplies map -> odom.
The original EKF configuration is in `planning_rpp/launch/planning.launch.py`.

For this first test, `run.yaml` does not initialise AMCL: use RViz's 2D Pose Estimate.
The first-path check remains a separate command and does not run concurrently.

1. Stop the standalone map server, Cartographer, first-path launch, and other Nav2
   launches. Keep robot drivers running. On the robot, source `robot_ws/install/setup.bash`
   and run `ros2 launch robot_init robot.launch.py` if not already running.
2. On the lab ROS machine, from the main project directory:

```bash
source /opt/ros/jazzy/setup.bash
cd student_ws
rosdep install --from-paths src/rb3302_nav --ignore-src -r -y
colcon build --symlink-install --packages-select rb3302_nav
source install/setup.bash
ros2 launch rb3302_nav localization.launch.py map:="$PWD/maps/rb3302_map.yaml"
```

`source` loads ROS and the built workspace into this terminal. `rosdep install`
installs declared dependencies (needed when dependencies change). `colcon build`
installs our package and launch/config files. `ros2 launch` starts our small wrapper;
`map:=...` passes the absolute map YAML path. Lifecycle activation is automatic.
If another EKF already publishes odom -> base_link, use `start_ekf:=false`.

3. Another ROS terminal, from the main project directory:

```bash
source /opt/ros/jazzy/setup.bash
rviz2 -d "$PWD/student_ws/src/planning_rpp/rviz/navigation.rviz"
```

`-d` loads the existing saved RViz display config. Keep Map, LaserScan, RobotModel,
and TF enabled. Disable GlobalPlan and WaypointMarkers for this test. The existing
AMCLParticles display may need a Nav2 ParticleCloud plugin on Jazzy; it is not needed
for the first pose/scan checks. To see the estimated pose directly, add the
PoseWithCovariance display for `/amcl_pose` (Transient Local durability).

4. Place base_link at a physically measured location and heading. Use 2D Pose Estimate:
   click its approximate location and drag in the direction the robot faces. This
   publishes an initial estimate, not proof of accuracy. Verify laser returns line
   up with several real mapped walls, not just a single wall.
5. Drive a short known distance slowly using the existing keyboard teleoperation
   tool, stop and measure. Then turn 90 degrees and return. Do not lift or drag the
   robot during a normal odometry/localisation trial.

Inspect estimated position/orientation with:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

This runs the TF inspection program: it prints the estimated transform from the
map frame to the robot frame, including position and orientation. Compare that
with independent tape measurements and a marked heading. To inspect AMCL's own
estimate and covariance, `ros2 topic echo /amcl_pose --once` prints one message.

Suggested initial acceptance targets (engineering targets, not assessment rules):

| Case | Evidence |
| --- | --- |
| Stationary at known pose | Several walls align; position error <=0.10 m, heading error <=10 degrees |
| Move a measured 0.6 m, then stop | Estimated displacement and final position agree with independent measurements |
| Turn 90 degrees, then return | Estimated heading follows; little estimated translation during the turn |
| Return to start | Pose returns within 0.10 m / 10 degrees without another initial-pose click |
| Repeat at three distinct positions, five trials each | Record errors and failures; don't infer success from one location |

The current letter centres are estimated from an image overlay, so they are not
independent ground truth for a centimetre-accuracy claim. Measure actual label/base
positions or distances to fixed walls. A small particle cloud/covariance alone can
be confidently wrong. Passing these trials supports measured accuracy, not universal
proof of localisation. Tune only after map, TF, sensor alignment and odometry checks.

Code/configuration syntax was checked locally. ROS startup and hardware localisation
must still be verified on the lab ROS 2 Jazzy machine; this Mac has no ROS runtime.

### RViz versus simulation

RViz displaying the saved map is visualisation only. AMCL also requires scans and
odometry/TF from a real robot or simulator. The map alone cannot provide them.
For a simulator that publishes `/scan`, odom -> base_link, sensor transforms and
`/clock`, pass `use_sim_time:=true start_ekf:=false` to the localisation launch.
RViz must also use simulation time, e.g. `--ros-args -p use_sim_time:=true`.
These arguments do not start a simulator. The simulator must use an environment
consistent with the saved map. Hardware `robot_init` is only for the real robot.
