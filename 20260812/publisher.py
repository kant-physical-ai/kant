import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')      # 노드 이름
        # 발행자 생성: (메시지타입, 토픽명, 큐깊이)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 0.05초(20Hz)마다 tick 호출
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info('발행 시작')

    def tick(self):
        # Twist 속도·자세·위치
        msg = Twist()
        msg.linear.x = 0.2      # 0.2 m/s 전진
        msg.angular.z = 0.1     # 약간 회전
        self.pub.publish(msg)

        # # LaserScan 센서 데이터
        # msg = LaserScan()
        # # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.ranges = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
        # self.pub.publish(msg)
        #
        # # Odometry 센서 데이터
        # msg = Odometry()
        # # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.pose.pose.position.x = 1.0
        # self.pub.publish(msg)
        #
        # # String
        # self.pub.publish('Hello ROS2!'ROS2)

def main():
    rclpy.init()
    node = VelocityPublisher()
    rclpy.spin(node)            # 콜백이 돌기 시작
    rclpy.shutdown()

if __name__ == '__main__':
    main()