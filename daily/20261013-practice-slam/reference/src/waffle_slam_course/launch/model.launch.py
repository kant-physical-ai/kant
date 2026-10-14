from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
import xacro

def generate_launch_description():
    share = Path(get_package_share_directory('turtlebot3_description'))
    urdf = xacro.process_file(str(share / 'urdf/turtlebot3_waffle.urdf')).toxml()
    return LaunchDescription([Node(package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': urdf}])])
