import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class EkfFusionNode(Node):
    def __init__(self):
        super().__init__('ekf_fusion_node')

        # 1. 하드웨어 센서 토픽 구독 (Subscribe)
        self.sub_wheel = self.create_subscription(Odometry, '/wheel_odom', self.wheel_cb, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        # 2. 융합된 단일 /odom 토픽 퍼블리셔 (Publish) - 시스템 전체에서 유일!
        self.pub_fused_odom = self.create_publisher(Odometry, '/odom', 10)

        # 3. odom -> base_link TF 브로드캐스터
        self.tf_broadcaster = TransformBroadcaster(self)

        # 내부 추정 상태 변수
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def wheel_cb(self, msg: Odometry):
        # 바퀴에서 이동 거리(선속도) 추출
        vx = msg.twist.twist.linear.x
        dt = 0.05  # 예시 주기 (20Hz)

        # 위치 단순 적분 (실제 EKF는 칼만 필터 가중치 행렬 연산 적용)
        self.x += vx * math.cos(self.yaw) * dt
        self.y += vx * math.sin(self.yaw) * dt

        self.publish_fused_odom_and_tf(msg.header.stamp)

    def imu_cb(self, msg: Imu):
        # IMU에서 회전 각속도 추출하여 Yaw 보정
        qz = msg.orientation.z
        qw = msg.orientation.w
        self.yaw = 2.0 * math.atan2(qz, qw)

    def publish_fused_odom_and_tf(self, stamp):
        # [출력 1] /odom 토픽 발행
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.pub_fused_odom.publish(odom_msg)

        # [출력 2] odom -> base_link TF 방송
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = odom_msg.pose.pose.orientation.z
        t.transform.rotation.w = odom_msg.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = EkfFusionNode()
    rclpy.spin(node)
    rclpy.shutdown()