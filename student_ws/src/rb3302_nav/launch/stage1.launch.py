"""Load the saved map and Nav2 planner, then draw the first path. No localisation."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from rb3302_nav.course import Course, Run


def start(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    course, run = Course(arg('course_file')), Run.load(arg('run_file'))
    course.validate(run)
    x, y, yaw = course.start_pose(run)
    params = ParameterFile(RewrittenYaml(
        source_file=arg('params_file'), root_key='', convert_types=True,
        param_rewrites={
            'global_costmap.global_costmap.ros__parameters.robot_radius': str(course.robot_radius),
            'global_costmap.global_costmap.ros__parameters.resolution': str(course.map.resolution),
        }))
    return [
        # Nav2's costmap needs a robot-frame transform even with use_start=True.
        # This separate frame is just the input pose, never a claim of localisation.
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', str(x), '--y', str(y), '--yaw', str(yaw),
                        '--frame-id', 'map', '--child-frame-id', 'planning_start']),
        # The map's origin is read from rb3302_map.yaml: [-1.134, -2.150, 0].
        # For that map, +X is image-right and +Y is image-up (yaw=0).
        # The planning_start transform above sets the input pose, NOT this origin.
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[params, {'yaml_filename': str(course.map.yaml_path)}], output='screen'),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[params], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='planning_lifecycle_manager', parameters=[{
                 'autostart': True, 'node_names': ['map_server', 'planner_server']}]),
        Node(package='rb3302_nav', executable='first_path', output='screen', parameters=[{
            'course_file': str(course.path), 'run_file': arg('run_file'),
            'output_file': arg('output_file')}]),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('rb3302_nav'))
    return LaunchDescription([
        DeclareLaunchArgument('course_file', description='Absolute path to course_letters.yaml'),
        DeclareLaunchArgument('run_file', default_value=str(share/'config'/'run.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(share/'config'/'stage1.yaml')),
        DeclareLaunchArgument('output_file', default_value='first_path.png'),
        OpaqueFunction(function=start),
    ])
