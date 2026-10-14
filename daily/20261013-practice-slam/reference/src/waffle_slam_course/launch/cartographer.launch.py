from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config = os.path.join(get_package_share_directory('waffle_slam_course'), 'config')
    return LaunchDescription([
        Node(package='cartographer_ros', executable='cartographer_node',
             parameters=[{'use_sim_time': True}],
             arguments=['-configuration_directory', config,
                        '-configuration_basename', 'waffle_2d.lua'],
             remappings=[('scan', '/scan'), ('odom', '/odom')], output='screen'),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             parameters=[{'use_sim_time': True}],
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
             output='screen')])
