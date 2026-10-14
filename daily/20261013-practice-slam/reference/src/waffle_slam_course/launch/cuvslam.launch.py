from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([ComposableNodeContainer(
        name='waffle_cuvslam', namespace='', package='rclcpp_components',
        executable='component_container', output='screen',
        composable_node_descriptions=[ComposableNode(
            name='visual_slam_node', package='isaac_ros_cuvslam',
            plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
            parameters=[{'use_sim_time': True, 'tracking_mode': 0, 'num_cameras': 2,
                         'base_frame': 'base_footprint', 'map_frame': 'map',
                         'odom_frame': 'odom', 'rectified_images': True,
                         'enable_localization_n_mapping': True,
                         'enable_image_denoising': True,
                         'enable_slam_visualization': True,
                         'enable_observations_view': True,
                         'enable_landmarks_view': True,
                         'publish_map_to_odom_tf': True,
                         'publish_odom_to_base_tf': True}],
            remappings=[('visual_slam/image_0', '/waffle/left/image_rect'),
                        ('visual_slam/camera_info_0', '/waffle/left/camera_info'),
                        ('visual_slam/image_1', '/waffle/right/image_rect'),
                        ('visual_slam/camera_info_1', '/waffle/right/camera_info')])])])
