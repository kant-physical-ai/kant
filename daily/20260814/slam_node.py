import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')

        # 1. EKF의 /odom 과 라이다 /scan 을 구독 (Subscribe)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # 2. /map 지도 퍼블리셔 (Publish)
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', 10)

        # 3. map -> odom TF 브로드캐스터
        self.tf_broadcaster = TransformBroadcaster(self)

        # 지도 설정 (20m x 20m, 5cm 해상도)
        self.resolution = 0.05
        self.width = 400
        self.height = 400
        self.grid_map = np.full((self.height, self.width), -1, dtype=np.int8)

        # EKF에서 받은 로봇 포즈 보관용
        self.robot_x = 0.0
        self.robot_y = 0.0

    def odom_cb(self, msg: Odometry):
        # EKF가 정제해준 /odom 토픽 수신
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def scan_cb(self, msg: LaserScan):
        # 1. 라이다 데이터 기반 그리드 지도 업데이팅 (생략/간소화)
        # ... (Scan Matching 및 Occupancy Grid 업데이트) ...

        # 2. /map 토픽 발행
        map_msg = OccupancyGrid()
        map_msg.header.stamp = msg.header.stamp
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        map_msg.data = self.grid_map.flatten().tolist()
        self.pub_map.publish(map_msg)

        # 3. map -> odom TF 방송 (전역 오차 보정 오프셋)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        # Scan Matching 결과 오차가 없다고 가정할 때 0,0,0
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()
    rclpy.spin(node)
    rclpy.shutdown()