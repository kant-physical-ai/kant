import time
from concurrent.futures import ThreadPoolExecutor
from rclpy.executors import MultiThreadedExecutor
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocitySubscriber(Node):
    def __init__(self):
        super().__init__('velocity_subscriber')
        # 구독자 생성: (메시지타입, 토픽명, 콜백, 큐깊이)
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.on_cmd, 10)
        # 무거운 작업을 수행할 워커 풀
        self.worker_pool = ThreadPoolExecutor(max_workers=2)

    def on_cmd(self, msg):       # 메시지가 올 때마다 호출
        self.get_logger().info(
            f'받음: 전진 {msg.linear.x:.2f} m/s, 회전 {msg.angular.z:.2f} rad/s')
        # 콜백은 즉시 반환: 무거운 작업은 워커 스레드로 위임
        self.worker_pool.submit(self.hard_work, msg)

    def hard_work(self, msg):
        # 이 함수는 ROS 콜백 스레드가 아니라 워커 스레드에서 실행됨
        time.sleep(11.1)

def main():
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(VelocitySubscriber())
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()