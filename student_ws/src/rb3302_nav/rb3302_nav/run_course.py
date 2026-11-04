"""Run four letter goals using the existing Nav2 stack and course coordinates."""
import argparse
import math
from pathlib import Path

from rb3302_nav.course import BEARINGS, LETTERS, Course, Run


def parse_inputs(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', required=True, type=str.upper, choices=LETTERS)
    parser.add_argument('--bearing', required=True, type=str.upper, choices=BEARINGS)
    parser.add_argument('--goals', required=True, nargs=4, type=str.upper, choices=LETTERS)
    parser.add_argument('--course', type=Path,
                        default=Path.home()/'rb3302_final_proj_code/student_ws/maps/course_letters.yaml')
    parser.add_argument('--check', action='store_true',
                        help='Validate and display inputs without ROS or movement')
    args = parser.parse_args(argv)
    run = Run(args.start, args.bearing, tuple(args.goals))
    try:
        course = Course(args.course)
        course.validate(run)
    except (ValueError, KeyError, OSError) as error:
        parser.error(str(error))
    return args, course, run


def pose(nav, x, y, yaw):
    from geometry_msgs.msg import PoseStamped
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = nav.get_clock().now().to_msg()
    msg.pose.position.x, msg.pose.position.y = float(x), float(y)
    msg.pose.orientation.z = math.sin(yaw/2)
    msg.pose.orientation.w = math.cos(yaw/2)
    return msg


def execute(nav, course, run):
    from nav2_simple_commander.robot_navigator import TaskResult
    x, y, yaw = course.start_pose(run)
    nav.setInitialPose(pose(nav, x, y, yaw))
    nav.waitUntilNav2Active()
    goals = []
    for index, letter in enumerate(run.goals, 1):
        gx, gy = course.letters[letter]
        # No arrival bearing is prescribed. Use the previous-to-next centre bearing;
        # this is a final orientation, not a restriction on the planned route.
        if math.hypot(gx-x, gy-y) > 1e-6:
            yaw = math.atan2(gy-y, gx-x)
        nav.get_logger().info(f'Goal {index}/4: {letter} ({gx:.3f}, {gy:.3f})')
        goals.append(pose(nav, gx, gy, yaw))
        x, y = gx, gy
    # The existing WaitAtWaypoint plugin owns the pause, including repeated letters.
    if nav.followWaypoints(goals) is False:
        raise RuntimeError('Waypoint mission was rejected')
    previous_index = None
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback is not None and feedback.current_waypoint != previous_index:
            previous_index = feedback.current_waypoint
            nav.get_logger().info(f'Navigating to landmark {previous_index+1}/4')
    if nav.getResult() != TaskResult.SUCCEEDED:
        raise RuntimeError('Waypoint mission did not succeed; check waypoint_follower logs')
    nav.get_logger().info('All four landmarks completed')


def main(argv=None):
    args, course, run = parse_inputs(argv)
    print(f'Start: {run.start}, bearing: {run.bearing}; goals: {", ".join(run.goals)}', flush=True)
    print(f'Start map pose (x, y, yaw rad): {course.start_pose(run)}', flush=True)
    if not course.calibrated:
        print('Letter coordinates are marked uncalibrated: verify label centres and heading_offset_deg.', flush=True)
    if args.check:
        print('Inputs checked. No movement requested.')
        return

    import rclpy
    from nav2_simple_commander.robot_navigator import BasicNavigator
    rclpy.init()
    nav = BasicNavigator()
    try:
        execute(nav, course, run)
    except (KeyboardInterrupt, RuntimeError) as error:
        nav.cancelTask()
        raise SystemExit(str(error) or 'Run interrupted')
    finally:
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
