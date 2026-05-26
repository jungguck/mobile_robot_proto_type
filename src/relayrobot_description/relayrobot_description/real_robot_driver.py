import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState # [추가됨] JointState 메시지 import 필수
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler 
import math
import numpy as np

# 드라이버 파일 (사용자 환경에 맞게 유지)
from .motor_drive_1 import MotorDriver 

class RealRobotDriver(Node):
    def __init__(self):
        super().__init__('real_robot_driver')

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
        
        # [추가됨] Joint State(바퀴 각도) 변수 초기화
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        
        # [중요] URDF 파일에 적힌 joint name과 똑같아야 합니다!
        self.joint_names = ['left_wheel_joint', 'right_wheel_joint']

        self.last_time = self.get_clock().now()

        # 3. Publisher & Subscriber
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # [추가됨] JointState Publisher 생성
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.br = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
         
        # 4. Timer (0.1초마다 실행)
        self.timer_period = 0.1 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("Real Robot Driver Started (Odom + JointStates)...")

    def cmd_vel_callback(self, msg):
        if not self.driver: return
        self.driver.drive(msg.linear.x, msg.angular.z)

    def timer_callback(self):
        if not self.driver: return

        # 1. Driver에서 실제 RPM 읽어오기 (가짜 값 아님!)
        rpm_L, rpm_R = self.driver.read_feedback()

        # 2. RPM -> 선속도(m/s) 변환 (님 코드의 계산식 유지)
        # (rpm_L / 6.0)이 실제 회전수라면, 이 식은 선속도(v)를 구하는 식입니다.
        vl = (rpm_L / 6.0) * (2 * math.pi * self.wheel_radius)
        vr = (rpm_R / 6.0) * (2 * math.pi * self.wheel_radius)

        # 3. 로봇 전체 속도 (Odom용)
        v = (vl + vr) / 2.0
        w = (vr - vl) / self.wheel_base

        # 4. 시간 간격(dt) 계산
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # -------------------------------------------------------------
        # [추가됨] Joint State 계산 (바퀴 회전 각도 적분)
        # -------------------------------------------------------------
        # 선속도(vl) = 반지름(r) * 각속도(omega) 이므로,
        # 각속도(rad/s) = 선속도(vl) / 반지름(r)
        
        ang_vel_L = vl / self.wheel_radius  # 왼쪽 바퀴 회전 속도 (rad/s)
        ang_vel_R = vr / self.wheel_radius  # 오른쪽 바퀴 회전 속도 (rad/s)

        # 적분: 현재 각도 += 각속도 * 시간
        self.left_wheel_angle += ang_vel_L * dt
        self.right_wheel_angle += ang_vel_R * dt

        # JointState 메시지 생성 및 발송
        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.name = self.joint_names # ['left_wheel_joint', 'right_wheel_joint']
        joint_msg.position = [self.left_wheel_angle, self.right_wheel_angle]
        joint_msg.velocity = [ang_vel_L, ang_vel_R] # 현재 회전 속도도 넣어줌
        
        self.joint_pub.publish(joint_msg)
        # -------------------------------------------------------------

        # 5. Odom 위치 적분 (기존 코드)
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_th = w * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th

        # 6. Odom 퍼블리시
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

        # Covariance 추가 (Cartographer 등에서 필요할 수 있음)
        odom.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
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