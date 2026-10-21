"""Our robot's EKF + Nav2's standard map server/AMCL bringup."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('rb3302_nav'))
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    params = str(share / 'config' / 'localization.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to saved map YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='True only when a simulator supplies /clock'),
        DeclareLaunchArgument('start_ekf', default_value='true',
                              description='Set false if the simulator or another EKF supplies odom->base_link'),
        # Same sensor selections as the provided planning_rpp launch.
        # Driver publishes /odom/raw but disables its odometry TF; this EKF owns it.
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[params, {'use_sim_time': ParameterValue(
                 LaunchConfiguration('use_sim_time'), value_type=bool)}],
             remappings=[('odometry/filtered', '/odom')],
             condition=IfCondition(LaunchConfiguration('start_ekf')), output='screen'),
        # This installed Nav2 launch starts nav2_map_server, nav2_amcl, and
        # lifecycle_manager_localization. We do not implement AMCL ourselves.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2 / 'launch' / 'localization_launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': params,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': 'true',     # Configure/activate automatically.
                'use_composition': 'False',
            }.items()),
    ])
