import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')

        # Nav2 / Navigator로부터 속도 명령 수신
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # 로봇 하드웨어 매개변수
        self.wheel_radius = 0.05
        self.wheel_base = 0.30

    def cmd_vel_cb(self, msg: Twist):
        # /cmd_vel 토픽에서 선속도(v)와 각속도(w) 추출
        v = msg.linear.x
        w = msg.angular.z

        # Inverse Kinematics (역운동학): v, w -> 좌/우 바퀴 속도(m/s) 변환
        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        # 바퀴 RPM 계산 (회전수/분)
        rpm_left = (v_left / (2.0 * 3.14159 * self.wheel_radius)) * 60.0
        rpm_right = (v_right / (2.0 * 3.14159 * self.wheel_radius)) * 60.0

        # [실제 구현] MCU(STM32/Arduino) 또는 모터 드라이버로 시리얼/CAN 제어 패킷 전송
        # ex: self.serial_port.write(f"M {rpm_left} {rpm_right}\n".encode())
        self.get_logger().info(f'Send to Motor Driver -> Left RPM: {rpm_left:.1f}, Right RPM: {rpm_right:.1f}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()