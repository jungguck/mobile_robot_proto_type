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
            self.driver = MotorDriver(port='/dev/ttyACM0') 
            if self.driver.ser is None:
                raise Exception("Serial connection returned None")
            self.get_logger().info("DDSM400 Motor Connected Successfully!")
        except Exception as e:
            self.get_logger().error(f"Motor connection failed: {e}")
            self.driver = None

        # 2. 로봇 파라미터
        self.wheel_radius = 0.035
        self.wheel_base = 0.2
        
        # Odom 위치 변수
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # IMU 관련 변수
        self.imu_yaw = 0.0
        self.imu_offset = None  # 시작 시점의 IMU 각도를 0으로 잡기 위함
        self.use_imu = False

        # Joint State 변수
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.joint_names = ['left_wheel_joint', 'right_wheel_joint']

        self.last_time = self.get_clock().now()

        # 3. Publisher & Subscriber
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.br = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # [추가] IMU 데이터 구독
        self.imu_sub = self.create_subscription(
            Imu,
            'ebimu_data',
            self.imu_callback,
            10
        )
         
        # 4. Timer (0.1초마다 실행)
        self.timer_period = 0.1 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("Real Robot Driver 260519 Started (Odom + IMU Fusion)...")

    def imu_callback(self, msg):
        # 쿼터니언을 오일러 각도로 변환
        orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        
        if self.imu_offset is None:
            self.imu_offset = yaw
            self.get_logger().info(f"IMU Offset initialized: {self.imu_offset}")
        
        # 시작 시점의 각도를 0으로 맞춤
        self.imu_yaw = yaw - self.imu_offset
        self.use_imu = True

    def cmd_vel_callback(self, msg):
        if not self.driver: return
        self.driver.drive(msg.linear.x, msg.angular.z)

    def timer_callback(self):
        if not self.driver: return

        # 1. RPM 읽기
        rpm_L, rpm_R = self.driver.read_feedback()

        # 2. RPM -> 선속도(m/s) 변환
        vl = (rpm_L / 6.0) * (2 * math.pi * self.wheel_radius)
        vr = (rpm_R / 6.0) * (2 * math.pi * self.wheel_radius)

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

        # 6. 위치 적분
        # 방향(Theta) 결정: IMU 데이터가 있으면 IMU를 쓰고, 없으면 바퀴 계산값을 씀
        if self.use_imu:
            self.theta = self.imu_yaw
        else:
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

        # IMU를 사용할 때는 신뢰도를 높이기 위해 Covariance 조정
        if self.use_imu:
            rot_cov = 0.01 # IMU가 있으면 회전 오차 작음
        else:
            rot_cov = 0.1  # 바퀴만 쓰면 회전 오차 큼

        odom.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, rot_cov
        ]
        odom.twist.covariance = odom.pose.covariance[:]

        self.odom_pub.publish(odom)

        # TF Broadcast
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.br.sendTransform(t)

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
