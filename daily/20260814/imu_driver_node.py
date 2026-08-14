import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuDriverNode(Node):
    def __init__(self):
        super().__init__('imu_driver_node')

        # IMU RAW 데이터 퍼블리셔 (EKF 입력용)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)

        # 50Hz 주기로 센서 데이터 수집
        self.timer = self.create_timer(0.02, self.read_imu_hardware)
        self.yaw = 0.0

    def read_imu_hardware(self):
        # [실제 구현] I2C/SPI/UART 통신으로 IMU 칩 데이터 레지스터 읽기
        dt = 0.02
        angular_velocity_z = 0.01  # 약한 회전 노이즈 포함 각속도

        self.yaw += angular_velocity_z * dt

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'  # 센서 물리 위치 프레임

        # Quaternion 자이로 각도 변환
        msg.orientation.z = math.sin(self.yaw / 2.0)
        msg.orientation.w = math.cos(self.yaw / 2.0)
        msg.angular_velocity.z = angular_velocity_z
        msg.linear_acceleration.z = 9.81  # 중력가속도

        self.pub_imu.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuDriverNode()
    rclpy.spin(node)
    rclpy.shutdown()