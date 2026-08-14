import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class WheelDriverNode(Node):
    def __init__(self):
        super().__init__('wheel_driver_node')

        # Raw 바퀴 오도메트리 토픽 퍼블리셔 (EKF 입력용)
        self.pub_wheel_odom = self.create_publisher(Odometry, '/wheel_odom', 10)

        # 20Hz 주기로 하드웨어 엔코더 값 읽기
        self.timer = self.create_timer(0.05, self.read_hardware_encoders)

        # 로봇 하드웨어 매개변수
        self.wheel_radius = 0.05  # 바퀴 반지름 5cm
        self.wheel_base = 0.30    # 두 바퀴 사이 거리 30cm

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def read_hardware_encoders(self):
        # [실제 구현] 시리얼/CAN 통신으로 MCU(STM32/Arduino)에서 엔코더 값 수신
        # 여기서는 0.2m/s 속도로 직진 중인 모의 신호 생성
        v_left = 0.2   # 좌측 바퀴 속도 (m/s)
        v_right = 0.2  # 우측 바퀴 속도 (m/s)

        # 차동 구동(Differential Drive) 기하학 계산
        v = (v_right + v_left) / 2.0
        w = (v_right - v_left) / self.wheel_base
        dt = 0.05

        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += w * dt

        # /wheel_odom 토픽 퍼블리시
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        self.pub_wheel_odom.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WheelDriverNode()
    rclpy.spin(node)
    rclpy.shutdown()