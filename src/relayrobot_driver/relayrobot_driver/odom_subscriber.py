import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        
        # 'odom' 토픽을 구독합니다. 메시지 타입은 Odometry입니다.
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10  # 큐 사이즈
        )
        self.get_logger().info("Odom Subscriber Node Started!")

    def odom_callback(self, msg):
        # 1. 위치 데이터 (X, Y) 추출
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # 2. 쿼터니언(Quaternion)을 오일러 각도(Yaw)로 변환
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # 라디안을 도(degree) 단위로 변환
        degree = math.degrees(yaw)

        # 3. 터미널에 출력
        self.get_logger().info(f"Pos: X={x:.2f}, Y={y:.2f} | Yaw: {degree:.1f}°")

def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()
    try:
        rclpy.spin(node) # 노드 실행 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()