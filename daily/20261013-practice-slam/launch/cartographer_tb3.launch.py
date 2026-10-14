"""Cartographer TB3 launch (slam_toolbox 없이 cartographer만)."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'turtlebot3_lds_2d.lua',
        ],
        remappings=[('scan', 'scan')],
        output='screen',
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        parameters=[{'use_sim_time': True}, {'resolution': 0.05}, {'publish_period_sec': 1.0}],
        output='screen',
    )

    return LaunchDescription([cartographer_node, occupancy_grid_node])
