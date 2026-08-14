import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
import math

class SimpleNavigatorNode(Node):
    def __init__(self):
        super().__init__('simple_navigator_node')

        # 1. 지도 정보 구독
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)

        # 2. 로봇 속도 제어 명령어 퍼블리셔 (Publish)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # 3. TF Listener 준비 (map -> base_link 변환 계산용)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 10Hz 주기로 제어 루프 실행
        self.timer = self.create_timer(0.1, self.control_loop)

        # 목표 위치 (map 좌표계 기준 x=2.0m, y=1.0m)
        self.target_x = 2.0
        self.target_y = 1.0

    def map_cb(self, msg: OccupancyGrid):
        # 지도 받아와서 장애물 회피 경로 생성에 활용
        pass

    def control_loop(self):
        try:
            # TF Tree를 조회하여 map 기준 base_link(로봇)의 현재 변환 좌표 조회
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y

            # 목표지점까지 남은 거리 계산
            dx = self.target_x - curr_x
            dy = self.target_y - curr_y
            dist = math.sqrt(dx**2 + dy**2)

            cmd = Twist()
            if dist > 0.1:  # 10cm 이내 접근 시 정지
                cmd.linear.x = 0.2  # 직진 속도 (0.2 m/s)
                cmd.angular.z = math.atan2(dy, dx) * 0.5  # 목표 방향으로 회전
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info('Goal Reached!')

            # 모터 컨트롤러로 속도 명령 발행
            self.pub_cmd_vel.publish(cmd)

        except Exception as e:
            # TF를 아직 못 가져왔을 때 예외 처리
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigatorNode()
    rclpy.spin(node)
    rclpy.shutdown()