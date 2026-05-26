import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry # odometry 메시지 형식을 ㄱ쓰기 ㅎ위해 가져옴 
import math # 수학 문제땜 시 필요 

class OdomListener(Node):
    def __init__(self):
    	# 노드이름 정하기 (우리는 아래와 같은 노드를 하나 만들겠다는 뜻 !!)
        super().__init__('odom_listener')
        # 리매핑된 odom 토픽 이름이 다르면 '/odom' 부분을 수정하세요
        # 구독 설정 
        self.subscription = self.create_subscription(
            Odometry,   # - Odometry: 메시지 타입 (어떤 형식의 편지인지)
            '/odom',    # - '/odom': 토픽 이름 (중요! 실제 로봇이 쏘는 이름과 같아야 함)
            self.listener_callback,
            10)  # - 10: 큐 사이즈 (데이터가 너무 빨리 오면 10개까지만 쌓아둠)

        self.get_logger().info('odom listener notde has been started')

    def listener_callback(self, msg):
        # 1. 위치 (X, Y) 가져오기
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # 2. 쿼터니언을 Euler Angle (Yaw)로 변환하기
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w

        # Yaw 계산 공식 (z축 회전)
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        yaw = math.atan2(siny_cosp, cosy_cosp) #  파이로 변환 
        
        # 라디안을 도로 변환 (필요시)
        yaw_deg = math.degrees(yaw)

        # 3. 깔끔하게 출력
        print(f"X: {x:.2f} | Y: {y:.2f} | Angle(Rad): {yaw:.2f} | Angle(Deg): {yaw_deg:.1f}")

def main(args=None):
    rclpy.init(args=args)   # 1. ROS 2 통신 기능을 켭니다.
    node = OdomListener()   # 2. 우리가 만든 노드 클래스를 생성합니다.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass # Ctrl+C를 누르면 여기로 와서 조용히 종료합니다.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()