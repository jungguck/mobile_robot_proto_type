import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quanternion_from_euler
import math
import numpy as np
# 주의: 저장하신 파일명이 motor_drive.py라면 아래처럼 고치세요 , / motor_driver_1 는 피드백 잇음 
# from motor_drive import MotorDriver   
from motor_drive import MotorDriver 

class RealRobotDriver(Node):
    def __init__(self):
        super().__init__('real_robot_driver')

        # 1. 모터 드라이버 객체 생성
        # 실제 포트가 /dev/ttyACM0 인지 확인 필수
        try:
        	# MotoDriver의 __init__이 실행되면서, 시리얼 연결 다됌 
            self.driver = MotorDriver(port='/dev/ttyACM0')

        except Exception as e:
            self.get_logger().error(f"Motor connection failed: {e}")
            return

        # 2. Subscriber 생성
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10  # 여기에 쉼표가 빠져서 에러가 났었습니다.
        )
        self.get_logger().info("Relay Robot Driver Node started")

    def cmd_vel_callback(self, msg):
        # 3. 메시지 파싱
        linear_x = msg.linear.x
        angular_z = msg.angular.z # 변수명을 맞춰야 합니다 (linear_z -> angular_z)
        
        # 4. 모터 드라이버 구동
        if self.driver:
            self.driver.drive(linear_x, angular_z)

    def stop_robot(self): # self가 빠져 있었습니다.
        if self.driver:
            self.driver.stop()
            self.driver.close()

# main 함수는 class 바깥으로 나와야 합니다! (들여쓰기 해제)
def main(args=None):
    rclpy.init(args=args)
    node = RealRobotDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()