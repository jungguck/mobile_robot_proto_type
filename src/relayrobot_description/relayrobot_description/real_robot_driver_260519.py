import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math
import numpy as np

# 드라이버 파일
from .motor_drive_1 import MotorDriver 

class RealRobotDriver260519(Node):
    def __init__(self):
        super().__init__('real_robot_driver_260519')

        # 1. 모터 드라이버 연결
        try:
            self.driver = MotorDriver(port='/dev/motor')
            if self.driver.ser is None:
                raise Exception("Serial connection returned None")
            self.get_logger().info("DDSM400 Motor Connected Successfully!")
        except Exception as e:
            self.get_logger().error(f"Motor connection failed: {e}")
            self.driver = None

        # 2. 로봇 파라미터
        self.wheel_radius = 0.05
        self.wheel_base = 0.165
        
        # Odom 위치 변수
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Joint State 변수
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.joint_names = ['left_wheel_joint', 'right_wheel_joint']

        self.last_time = self.get_clock().now()

        # 3. Publisher & Subscriber
        # EKF를 사용할 것이므로 odom 토픽 이름은 odom_unfiltered로 변경하거나 그대로 둬도 됨.
        # robot_localization에서 odom을 구독하도록 설정했음. 여기선 그냥 odom 또는 raw_odom 발행.
        # launch.py에서 EKF가 odometry/filtered를 odom으로 리맵핑함. 
        # 드라이버는 'odom_raw' 또는 'wheel/odom' 등으로 발행하고, EKF가 그걸 구독해야 충돌이 없음.
        # 하지만 ekf.yaml을 보니 odom0: /odom 으로 설정되어 있음.
        # 즉 EKF가 /odom을 구독하고, odometry/filtered를 뱉으면, launch에서 odom으로 리맵하면 무한루프.
        # EKF가 odom_raw를 구독하게 하거나, 드라이버가 odom_raw를 쏘게 하고 ekf.yaml을 고쳐야 함.
        # 일단 여기선 'odom_raw'로 쏘도록 변경하자.
        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
         
        # 4. Timer (0.1초마다 실행)
        self.timer_period = 0.1 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("Real Robot Driver Started (Wheel Odom Only)...")

    def cmd_vel_callback(self, msg):
        if not self.driver: return
        self.driver.drive(msg.linear.x, msg.angular.z)

    def timer_callback(self):
        if not self.driver: return

        # 1. RPM 읽기
        rpm_L, rpm_R = self.driver.read_feedback()
        rpm_L = -rpm_L  # 왼쪽 모터는 drive()에서 -cmd로 전송하므로 피드백도 부호 반전 필요

        # 2. RPM -> 선속도(m/s) 변환
        # spd 피드백은 실제 RPM의 10배 (cmd 100 = 10 RPM), 따라서 /600 = /60/10
        vl = (rpm_L / 600.0) * (2 * math.pi * self.wheel_radius)
        vr = (rpm_R / 600.0) * (2 * math.pi * self.wheel_radius)

        # 3. 로봇 속도
        v = (vl + vr) / 2.0
        w_enc = (vr - vl) / self.wheel_base # 엔코더 기반 각속도

        # 4. 시간 간격(dt)
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 5. Joint State 계산 (시각화용)
        ang_vel_L = vl / self.wheel_radius
        ang_vel_R = vr / self.wheel_radius
        self.left_wheel_angle += ang_vel_L * dt
        self.right_wheel_angle += ang_vel_R * dt

        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = [self.left_wheel_angle, self.right_wheel_angle]
        joint_msg.velocity = [ang_vel_L, ang_vel_R]
        self.joint_pub.publish(joint_msg)

        # 6. 위치 적분 (바퀴 오도메트리 전용)
        self.theta += w_enc * dt

        # X, Y는 바퀴 속도와 현재 방향(Theta)으로 계산
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        # 7. Odom 퍼블리시
        self.publish_odom(v, w_enc)

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

        # 바퀴 오차 설정
        odom.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]
        odom.twist.covariance = odom.pose.covariance[:]

        self.odom_pub.publish(odom)

        # TF Broadcast 제거 (ekf_node가 담당)

    def stop_robot(self):
        if self.driver:
            self.driver.stop()
            self.driver.close()

def main(args=None):
    rclpy.init(args=args)
    node = RealRobotDriver260519()
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
