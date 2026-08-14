import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. URDF (my_robot_description) 실행 스크립트 불러오기 ⭐ [핵심!]
    desc_pkg_dir = get_package_share_directory('my_robot_description')
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg_dir, 'launch', 'display.launch.py')
        )
    )

    # 2. 내 프로젝트 제어 노드
    my_control_node = Node(
        package='my_robot_control',
        executable='my_controller_node',
        output='screen'
    )

    # 3. 드라이버 노드들...
    # lidar_node = Node(...)

    # 무조건 같이 실행되도록 리스트로 묶어서 반환!
    return LaunchDescription([
        description_launch,  # URDF & robot_state_publisher 자동 실행
        my_control_node,     # 내 알고리즘 실행
    ])










# 각노드에 방어코드넣을수있다
# import rclpy
# from rclpy.node import Node
# from tf2_ros import Buffer, TransformListener, TransformException
#
# class MyNode(Node):
#     def __init__(self):
#         super().__init__('my_node')
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
#
#         # 1초마다 TF 조회 테스트
#         self.timer = self.create_timer(1.0, self.get_tf)
#
#     def get_tf(self):
#         try:
#             # TF 가져오기 시도
#             now = rclpy.time.Time()
#             trans = self.tf_buffer.lookup_transform('base_link', 'laser_frame', now)
#             self.get_logger().info(f"TF 성공! laser_x: {trans.transform.translation.x}")
#         except TransformException as ex:
#             # URDF가 안 떠서 TF를 못 가져와도 다운되지 않고 경고만 출력!
#             self.get_logger().warn(f"TF 조회 실패 (URDF가 켜져 있는지 확인하세요): {ex}")