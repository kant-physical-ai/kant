"""Jazzy TurtleBot3 assets with the installed Lyrical ros_gz / Gazebo runtime."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    gz = get_package_share_directory('ros_gz_sim')
    course = get_package_share_directory('waffle_slam_course')

    def include(folder, file, arguments, condition=None):
        kwargs = {'condition': condition} if condition is not None else {}
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(folder, 'launch', file)),
            launch_arguments=arguments.items(), **kwargs)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false', choices=['true', 'false']),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(course, 'models')),
        include(gz, 'gz_sim.launch.py', {
            'gz_args': ['-r -s --headless-rendering -v 2 ', os.path.join(course, 'worlds', 'twin_closed_maze.sdf')],
            'on_exit_shutdown': 'true'}),
        include(gz, 'gz_sim.launch.py', {'gz_args': '-g -v 2'}, IfCondition(LaunchConfiguration('gui'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True,
                          'robot_description': Path(tb3, 'urdf', 'turtlebot3_waffle.urdf').read_text()}]),
        include(tb3, 'spawn_turtlebot3.launch.py', {'x_pose': '-1.835', 'y_pose': '2.757'}),
    ])
