"""
바퀴 오도메트리 캘리브레이션 도구.

cmd_vel로 정해진 동작을 시킨 뒤 /odom_raw가 적분한 값과
줄자로 잰 실제 값을 비교해서 wheel_radius / wheel_base 보정치를 뽑는다.

사용법
------
STAGE 2 (직진 → wheel_radius 보정):
    ros2 run relayrobot_description odom_calibrate --ros-args \
        -p mode:=straight -p speed:=0.15 -p duration:=10.0

    로봇이 멈추면 실제 이동거리를 줄자로 재서 다시 실행:
        ... -p measured:=1.42

STAGE 3 (제자리 회전 → wheel_base 보정):
    ros2 run relayrobot_description odom_calibrate --ros-args \
        -p mode:=rotate -p speed:=0.6 -p duration:=20.0 -p measured:=1080.0
    (measured = 바닥 테이프 기준으로 실제로 돈 각도, 단위 deg)
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomCalibrate(Node):

    def __init__(self):
        super().__init__('odom_calibrate')

        self.declare_parameter('mode', 'straight')   # straight | rotate
        self.declare_parameter('speed', 0.15)        # m/s 또는 rad/s
        self.declare_parameter('duration', 10.0)     # 초
        self.declare_parameter('measured', 0.0)      # 실측값 (m 또는 deg). 0이면 비교 생략
        self.declare_parameter('odom_topic', '/odom_raw')

        self.mode     = self.get_parameter('mode').value
        self.speed    = self.get_parameter('speed').value
        self.duration = self.get_parameter('duration').value
        self.measured = self.get_parameter('measured').value
        topic         = self.get_parameter('odom_topic').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, topic, self.odom_cb, 10)

        # 적분 상태
        self.have_first = False
        self.x0 = self.y0 = 0.0
        self.x = self.y = 0.0
        self.path_len = 0.0          # 실제 주행 경로 길이 (직선거리와 구분)
        self.prev_x = self.prev_y = 0.0
        self.yaw_unwrapped = 0.0     # ±pi 래핑 없는 누적 yaw
        self.prev_yaw = 0.0
        self.yaw0 = 0.0
        self.samples = 0

        self.started = False
        self.t_start = None

        self.get_logger().info(
            f"mode={self.mode}, speed={self.speed}, duration={self.duration}s, "
            f"odom={topic}"
        )
        self.get_logger().info("odom 첫 수신 대기 중... (드라이버 노드가 떠 있어야 합니다)")

        self.timer = self.create_timer(0.05, self.tick)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.samples += 1

        if not self.have_first:
            self.have_first = True
            self.x0, self.y0 = p.x, p.y
            self.prev_x, self.prev_y = p.x, p.y
            self.yaw0 = yaw
            self.prev_yaw = yaw
            return

        # 경로 길이 누적
        self.path_len += math.hypot(p.x - self.prev_x, p.y - self.prev_y)
        self.prev_x, self.prev_y = p.x, p.y
        self.x, self.y = p.x, p.y

        # yaw 언래핑 (여러 바퀴 회전을 그대로 누적)
        d = yaw - self.prev_yaw
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        self.yaw_unwrapped += d
        self.prev_yaw = yaw

    def tick(self):
        if not self.have_first:
            return

        now = self.get_clock().now()
        if not self.started:
            self.started = True
            self.t_start = now
            self.get_logger().info(">>> 주행 시작. 바닥에 시작점 표시해 두세요.")

        elapsed = (now - self.t_start).nanoseconds / 1e9

        if elapsed < self.duration:
            cmd = Twist()
            if self.mode == 'rotate':
                cmd.angular.z = self.speed
            else:
                cmd.linear.x = self.speed
            self.cmd_pub.publish(cmd)
            return

        # 정지 후 결과 출력
        self.cmd_pub.publish(Twist())
        self.timer.cancel()
        self.report(elapsed)
        rclpy.shutdown()

    def report(self, elapsed):
        straight = math.hypot(self.x - self.x0, self.y - self.y0)
        yaw_deg = math.degrees(self.yaw_unwrapped)

        print("\n" + "=" * 58)
        print(f" 모드          : {self.mode}")
        print(f" 주행 시간     : {elapsed:.2f} s   (odom 샘플 {self.samples}개)")
        print(f" odom 직선거리 : {straight:.4f} m")
        print(f" odom 경로길이 : {self.path_len:.4f} m")
        print(f" odom 회전각   : {yaw_deg:.2f} deg")

        if self.samples < 10:
            print("\n [경고] odom 샘플이 너무 적습니다. 드라이버가 피드백을")
            print("        못 읽고 있을 수 있습니다 (rpm이 계속 0인지 확인).")

        if self.measured == 0.0:
            print("\n 실제 값을 재서 -p measured:=<값> 으로 다시 실행하면")
            print(" 보정 계수를 계산해 드립니다.")
            print("=" * 58 + "\n")
            return

        print("-" * 58)
        if self.mode == 'rotate':
            if abs(yaw_deg) < 1e-6:
                print(" odom 회전각이 0이라 보정 계산 불가.")
            else:
                k = self.measured / yaw_deg
                print(f" 실측 회전각   : {self.measured:.2f} deg")
                print(f" 보정 계수     : {k:.4f}")
                print(f" → wheel_base 를 현재값 × {k:.4f} 로 바꾸세요.")
                print(f"   (예: 0.22 → {0.22 * k:.4f})")
        else:
            if straight < 1e-6:
                print(" odom 거리가 0이라 보정 계산 불가.")
            else:
                k = self.measured / straight
                print(f" 실측 이동거리 : {self.measured:.4f} m")
                print(f" 보정 계수     : {k:.4f}")
                print(f" → wheel_radius 를 현재값 × {k:.4f} 로 바꾸세요.")
                print(f"   (예: 0.0325 → {0.0325 * k:.5f})")
                print(f" 회전 드리프트 : {yaw_deg:.2f} deg "
                      f"(직진인데 크게 틀어지면 좌우 바퀴 반경이 다른 것)")
        print("=" * 58 + "\n")

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = OdomCalibrate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
