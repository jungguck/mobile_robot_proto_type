import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler 
import math
import numpy as np

# 드라이버 파일
from motor_drive_1 import MotorDriver 

class RealRobotDriver(Node):
    def __init__(self):
        super().__init__('real_robot_driver')

        # 1. 모터 드라이버 연결
        try:
            self.driver = MotorDriver(port='/dev/ttyACM0') 
            # 연결 확인
            if self.driver.ser is None:
                raise Exception("Serial connection returned None")
            self.get_logger().info("DDSM400 Motor Connected Successfully!")
        except Exception as e:
            self.get_logger().error(f"Motor connection failed: {e}")
            self.driver = None

        # 2. 로봇 파라미터
        self.wheel_radius = 0.035
        self.wheel_base = 0.2
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # 3. Publisher & Subscriber
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.br = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 4. Timer (0.1초마다 무조건 실행됨!)
        self.timer_period = 0.1 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("Odom Publisher Started...")

    def cmd_vel_callback(self, msg):
        if not self.driver: return
        self.driver.drive(msg.linear.x, msg.angular.z)

    def timer_callback(self):
        if not self.driver: return

        # 1. Driver에서 현재 RPM 읽어오기
        rpm_L, rpm_R = self.driver.read_feedback()

        # [디버깅] 여기서 0이 나오는지 숫자가 나오는지 눈으로 확인하세요!
        # 값이 계속 0이면 모터 드라이버의 JSON 키값 문제임.
        if rpm_L != 0 or rpm_R != 0:
            print(f"RPM Check -> L: {rpm_L}, R: {rpm_R}") 

        # 2. RPM -> 속도(m/s) 변환
        # 원래 60을 나눠야하지만 속도값(rpm_L,rpm_R)이 약 10배 작아야 RPM 값임 그래서 60 -> 600
        vl = (rpm_L / 600.0) * (2 * math.pi * self.wheel_radius)
        vr = (rpm_R / 600.0) * (2 * math.pi * self.wheel_radius)

        # 3. 로봇 전체 속도
        v = (vl + vr) / 2.0
        w = (vr - vl) / self.wheel_base

        # 4. 위치 적분
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_th = w * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th

        # 5. 퍼블리시
        self.publish_odom(v, w)

    def publish_odom(self, v, w):
        q = quaternion_from_euler(0, 0, self.theta)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # [수정] Covariance (공분산) 채우기 - Rviz 경고 방지용
        # 대각선 성분에 작은 값을 넣어줍니다. (x, y, z, roll, pitch, yaw 순서)
        odom.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]
        odom.twist.covariance = odom.pose.covariance[:] # 똑같이 복사

        self.odom_pub.publish(odom)

        # TF Broadcast
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = odom.pose.pose.orientation
        self.br.sendTransform(t)

    def stop_robot(self):
        if self.driver:
            self.driver.stop()
            self.driver.close()

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