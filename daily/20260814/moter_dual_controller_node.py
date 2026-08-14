import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class MotorControlNode(Node):

    def __init__(self):
        super().__init__('motor_control_node')

        # ==========================================
        # 1. 로봇 하드웨어 물리 제원 (URDF 수치와 일치해야 함)
        # ==========================================
        self.wheel_radius = 0.05  # 바퀴 반지름 (5cm = 0.05m)
        self.wheel_separation = 0.30  # 좌우 바퀴 간격 (30cm = 0.30m)

        # 모터 현재 위치(각도, rad) 저장 변수
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # ==========================================
        # 2. Publisher & Subscriber 설정
        # ==========================================
        # Nav2 / Teleop 노드로부터 속도 명령(/cmd_vel)을 구독
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # robot_state_publisher에게 바퀴 회전 각도를 전달할 /joint_states 발행
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # 20Hz (0.05초)마다 하드웨어 주기적 제어 및 JointState 발행 타이머 실행
        self.timer = self.create_timer(0.05, self.control_loop)

        # 실제 모터 시리얼/CAN 통신 연결 (가상 예시)
        self.init_hardware_communication()

        self.get_logger().info('Motor Control Node has been started.')

    def init_hardware_communication(self):
        """실제 하드웨어(Arduino, STM32, CAN 통신 등)와 연결을 초기화하는 함수"""
        # 예: self.serial = serial.Serial('/dev/ttyUSB0', 115200)
        self.get_logger().info('Hardware communication initialized.')

    def cmd_vel_callback(self, msg: Twist):
        """/cmd_vel (v, w) 메시지를 받아 개별 바퀴 속도(rad/s)로 계산하는 역기구학 함수"""
        v = msg.linear.x  # 선속도 (m/s)
        w = msg.angular.z  # 각속도 (rad/s)

        # ----------------------------------------------------
        # 3. 차동 구동 역기구학 (Inverse Kinematics) 공식
        # ----------------------------------------------------
        # 각 바퀴의 선속도 (m/s)
        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)

        # 선속도를 바퀴의 각속도 (rad/s)로 변환: w_wheel = v / r
        w_left_rad_s = v_left / self.wheel_radius
        w_right_rad_s = v_right / self.wheel_radius

        # ----------------------------------------------------
        # 4. 하드웨어 모터 드라이버로 속도 명령 전달
        # ----------------------------------------------------
        self.send_to_motor_hardware(w_left_rad_s, w_right_rad_s)

    def send_to_motor_hardware(self, left_speed_rad_s, right_speed_rad_s):
        """실제 모터 드라이버 MCU로 패킷을 쏘는 부분"""
        # 예시: RPM 단위로 변환 후 시리얼 전송
        left_rpm = left_speed_rad_s * 9.54929
        right_rpm = right_speed_rad_s * 9.54929

        # self.serial.write(f"M {left_rpm:.1f} {right_rpm:.1f}\n".encode())
        self.get_logger().debug(
            f'Target Speed -> Left: {left_rpm:.1f} RPM, Right: {right_rpm:.1f} RPM'
        )

    def control_loop(self):
        """20Hz 주기로 실행: 실제 바퀴 각도를 읽어서 /joint_states로 쏘아줌"""
        # 1. 실제 하드웨어 엔코더 값 읽어오기 (여기서는 가상으로 적분 처리)
        # 실제 구현 시: left_deg, right_deg = self.read_encoder_from_mcu()
        dt = 0.05  # 주기 0.05초
        # (테스트용 가상 값 적분)
        self.left_wheel_pos += 0.0  # 실제로는 모터 엔코더 측정값 입력
        self.right_wheel_pos += 0.0

        # 2. ROS 2 /joint_states 토픽으로 발행 -> robot_state_publisher가 받아서 3D 화면(RViz) 바퀴 회전
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]

        self.joint_state_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()