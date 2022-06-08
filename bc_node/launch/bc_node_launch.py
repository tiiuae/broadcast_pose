# BC Node launch file
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchIntrospector
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description(argv=sys.argv[1:]):
    # ======= Launch args =======

    config_folder = LaunchConfiguration('config_folder', default='')
    DeclareLaunchArgument(
        'config_folder',
        default_value=config_folder,
        description='Path to configuration folder')

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='Whether ros2 simulation time is used')

    log_level = LaunchConfiguration('log_level', default='info')
    DeclareLaunchArgument(
        'log_level',
        default_value=log_level,
        description='Set the ROS2 default logging level (default info).')
    loglevel_cmdline = ['--ros-args', '--log-level', log_level]

    # ======= Config =======
    # Visualizer tree
    show_launch_summary = False

    # ======= Config files for the nodes =======

    # == Package share folders
    bc_node_params = os.path.join(
        get_package_share_directory('bc_node'), 'params', "bc_node.yaml")

    # ======= Declare nodes =======
    # BC node
    bc_node = Node(
        package='bc_node',
        executable='bc_node',
        name='bc_node',
        output='screen',
        parameters=[bc_node_params] +
        [{'use_sim_time': use_sim_time}],
        arguments=loglevel_cmdline
    )

    # ======= Add event handlers to the launch description
    ld = LaunchDescription()

    # Add the actions to the launch description.
    # The order they are added reflects the order in which they will be executed.
    ld.add_action(bc_node)

    if show_launch_summary:
        print(LaunchIntrospector().format_launch_description(ld))

    return ld
