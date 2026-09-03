import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math
import numpy as np

from .motor_drive_1 import MotorDriver


class RealRobotDriver260519(Node):
    """
    역할: 모터 시리얼 통신 + 바퀴 오도메트리 계산

    발행 토픽:
      /odom_raw    → EKF가 구독해서 IMU와 융합 → /odom 생성
      /joint_states → robot_state_publisher가 구독해서 RViz 바퀴 시각화
    구독 토픽:
      /cmd_vel     → MPC 또는 teleop에서 속도 명령 수신
    """

    def __init__(self):
        super().__init__('real_robot_driver_260519')

        # 실측: 바퀴 지름 65mm(r=0.0325), 양 바퀴 중심간 거리 220mm
        # 캘리브레이션 중에는 재빌드 없이 --ros-args -p 로 바로 바꿀 수 있도록 파라미터화
        self.declare_parameter('port', '/dev/motor')
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_base', 0.22)
        self.declare_parameter('rpm_scale', 600.0)

        port              = self.get_parameter('port').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base   = self.get_parameter('wheel_base').value
        self.rpm_scale    = self.get_parameter('rpm_scale').value

        try:
            # 명령/오도메트리가 같은 기구학 상수를 쓰도록 드라이버에 그대로 주입
            self.driver = MotorDriver(port=port,
                                      wheel_radius=self.wheel_radius,
                                      wheel_base=self.wheel_base)
            if self.driver.ser is None:
                raise Exception("Serial connection returned None")
            self.get_logger().info("DDSM400 Motor Connected Successfully!")
        except Exception as e:
            self.get_logger().error(f"Motor connection failed: {e}")
            self.driver = None

        self.get_logger().info(
            f"kinematics: r={self.wheel_radius} m, base={self.wheel_base} m, "
            f"rpm_scale={self.rpm_scale}"
        )

        # 적분으로 누적되는 위치 (노드 시작 시 0으로 초기화)
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        self.left_wheel_angle  = 0.0
        self.right_wheel_angle = 0.0
        self.joint_names = ['left_wheel_joint', 'right_wheel_joint']

        self.last_time = self.get_clock().now()

        # /odom_raw: 순수 바퀴 인코더 odom. EKF가 이걸 받아 IMU와 융합 → /odom 출력
        # /odom을 직접 발행하지 않는 이유: EKF 출력과 토픽 이름 충돌 방지
        self.odom_pub  = self.create_publisher(Odometry,   'odom_raw',     10)
        self.joint_pub = self.create_publisher(JointState,  '/joint_states', 10)

        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )

        # 10Hz: 시리얼 레이턴시(10ms)와 맞춘 주기
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Real Robot Driver Started (Wheel Odom Only)...")

    def cmd_vel_callback(self, msg):
        if not self.driver:
            return
        self.driver.drive(msg.linear.x, msg.angular.z)

    def timer_callback(self):
        if not self.driver:
            return

        # read_feedback() 이 하드웨어 장착 부호(DIR_L/DIR_R)를 이미 흡수해서
        # "+ = 로봇 전진" 으로 돌려준다. 여기서 다시 뒤집으면 안 된다.
        rpm_L, rpm_R = self.driver.read_feedback()

        # spd 는 RPM 이 아니라 0.1RPM 단위다. 즉 10 RPM = spd 100.
        #
        #   spd ──÷10──▶ RPM ──÷60──▶ rev/s ──×2πr──▶ m/s
        #       └──────── rpm_scale = 10 × 60 = 600 ────────┘
        #
        # 명령 쪽 calculate_rpms() 의 ×600 과 정확히 서로의 역연산이므로 어긋나지 않는다.
        #   0.1 m/s ÷ 2πr(0.2042) = 0.4897 rev/s ×600 → cmd 293
        #   spd 293 ÷ 600 = 0.4883 rev/s ×0.2042    → 0.0997 m/s  (왕복 일치)
        #
        # 2026-09-03 확정값. 이 값을 조정하지 말 것. (docs/DEBUG_LOG_2026-09-03.md 2절)
        # odom 거리가 안 맞으면 이 팩터가 아니라 유령 거리(피드백 stale)를 먼저 의심할 것.
        # 결과: 바퀴 선속도(m/s)
        vl = (rpm_L / self.rpm_scale) * (2 * math.pi * self.wheel_radius)
        vr = (rpm_R / self.rpm_scale) * (2 * math.pi * self.wheel_radius)

        # 차동 구동 기구학: 로봇 중심 선속도 / 각속도
        v     = (vl + vr) / 2.0
        w_enc = (vr - vl) / self.wheel_base

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # RViz 바퀴 회전 시각화용 (제어에는 안 쓰임)
        ang_vel_L = vl / self.wheel_radius
        ang_vel_R = vr / self.wheel_radius
        self.left_wheel_angle  += ang_vel_L * dt
        self.right_wheel_angle += ang_vel_R * dt

        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.name     = self.joint_names
        joint_msg.position = [self.left_wheel_angle, self.right_wheel_angle]
        joint_msg.velocity = [ang_vel_L, ang_vel_R]
        self.joint_pub.publish(joint_msg)

        # 오일러 적분: 현재 방향(theta) 기준으로 x, y 누적
        self.theta += w_enc * dt
        self.x     += v * math.cos(self.theta) * dt
        self.y     += v * math.sin(self.theta) * dt

        self.publish_odom(v, w_enc)

    def publish_odom(self, v, w):
        q = quaternion_from_euler(0, 0, self.theta)

        odom = Odometry()
        odom.header.stamp    = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"       # 기준 좌표계
        odom.child_frame_id  = "base_link"  # 로봇 몸통

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w

        # covariance: EKF에게 "이 데이터를 얼마나 믿어도 되는지" 알려주는 값
        # 값이 클수록 불확실 → EKF가 이 센서를 덜 믿고 IMU를 더 반영
        odom.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0,  0.1,
        ]
        odom.twist.covariance = odom.pose.covariance[:]

        self.odom_pub.publish(odom)

        # TF(odom→base_link)는 ekf_node가 담당 — 여기서 발행하면 이중 충돌

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
