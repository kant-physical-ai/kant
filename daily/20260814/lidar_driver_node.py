import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarDriverNode(Node):
    def __init__(self):
        super().__init__('lidar_driver_node')

        # 라이다 스캔 데이터 퍼블리셔 (SLAM 입력용)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)

        # 10Hz 주기로 1회전 스캔 데이터 생성
        self.timer = self.create_timer(0.1, self.read_lidar_hardware)

    def read_lidar_hardware(self):
        # [실제 구현] 시리얼/USB 패킷을 디코딩하여 각도별 거리 배열 수신
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'  # 라이다 센서 위치 프레임

        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = math.pi / 180.0  # 1도 해상도 (총 360개 샘플)
        msg.range_min = 0.15
        msg.range_max = 12.0

        # 가상의 벽 거리 3m로 샘플링 배열 구성
        msg.ranges = [3.0] * 360

        self.pub_scan.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarDriverNode()
    rclpy.spin(node)
    rclpy.shutdown()