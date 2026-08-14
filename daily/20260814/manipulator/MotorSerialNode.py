import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import math

class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')

        # 1. 시리얼 포트 연결 (환경에 맞게 수정)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        # 2. /joint_states 퍼블리셔 생성
        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', 10)

        # 3. 20Hz (0.05초) 간격으로 시리얼 읽고 토픽 발행
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            # 예: 시리얼로 "45.0,90.0,-30.0" 형태로 값이 들어온다고 가정
            data = line.split(',')

            if len(data) == 3:
                # Degree -> Radian 변환
                rad1 = math.radians(float(data[0]))
                rad2 = math.radians(float(data[1]))
                rad3 = math.radians(float(data[2]))

                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = ['joint_1', 'joint_2', 'joint_3'] # URDF와 일치
                msg.position = [rad1, rad2, rad3]

                self.pub_joint_state.publish(msg)

def main():
    rclpy.init()
    node = MotorSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()