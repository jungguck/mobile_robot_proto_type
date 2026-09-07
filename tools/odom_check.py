#!/usr/bin/env python3
"""
odom_check.py — PC(개발기)에서 젯슨의 오도메트리를 원격 진단/캘리브레이션하는 노드

[무엇을 하나]
  젯슨이 발행하는 토픽을 DDS 로 구독해서(같은 ROS_DOMAIN_ID=0),
  "명령(cmd_vel) vs 바퀴 오도메트리(/odom_raw) vs IMU(/ebimu_data) vs EKF(/odom)"
  네 소스를 실시간으로 나란히 비교한다. 로봇을 조금 굴려보고 아래 두 비율이
  1.0 에서 벗어나면 기구학 상수가 틀린 것이다.

    ① 선속도 스케일  = odom_raw 이동거리 / 명령 이동거리   → 벗어나면 rpm_scale / wheel_radius
    ② 회전 스케일    = odom_raw Δθ       / IMU Δyaw        → 벗어나면 wheel_base

  ②는 IMU 절대 yaw 를 기준으로 삼으므로 줄자 없이도 회전 오차를 잡아낸다.
  (이 IMU 는 자이로를 안 주고 절대 자세만 준다 — ekf.yaml 주석 참고)

[이 노드는 젯슨이 아니라 PC 에서 돈다]
  전부 표준 메시지라 Jazzy(PC) ↔ Humble(젯슨) cross-distro 로 그냥 받아진다.
  젯슨 워크스페이스에 빌드할 필요 없다.

[실행]
  # 0) 젯슨에서 로봇 기동 (라이다는 꺼도 됨)
  ssh robot 'ros2 launch relayrobot_description real_robot_260519.launch.py use_lidar:=false'

  # 1) PC 에서 이 노드 실행
  #    ⚠️ 이 PC 는 conda(파이썬3.13) 가 기본 활성화라 rclpy(3.12) 가 깨진다.
  #       반드시 conda 를 끄고 시스템 파이썬으로 돌려야 한다.
  conda deactivate                # which python3 가 /usr/bin/python3 가 될 때까지 (base 면 한 번 더)
  ros_setup                       # jazzy + 워크스페이스 (ROS_DOMAIN_ID=0 은 .bashrc 에서 이미 설정)
  python3 tools/odom_check.py     # = /usr/bin/python3. conda 켜져 있으면 ModuleNotFoundError: rclpy._rclpy_pybind11

  # 2) PC 에서 로봇을 조금씩 굴린다 (다른 터미널)
  ros2 run teleop_twist_keyboard teleop_twist_keyboard   # 없으면: sudo apt install ros-jazzy-teleop-twist-keyboard
  #  또는 한 번씩:  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once

  # 3) 캘리브레이션 절차
  #  - 직진 테스트: 바닥에 1.0 m 표시 → reset → 그 지점까지 굴림 → 정지 → '선속도 스케일' 읽기
  #  - 회전 테스트: reset → 제자리 회전(예: angular.z 만) → '회전 스케일' 읽기 (줄자 불필요)
  ros2 service call /odom_check/reset std_srvs/srv/Trigger    # 누적값 0 으로

⚠️ 안전: 로봇이 실제로 움직인다. 바퀴 띄우거나 주변 비우고 할 것.
   비상정지:  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


def yaw_from_quaternion(q) -> float:
    """쿼터니언 → yaw(rad). tf_transformations 의존 없이 직접 계산한다
    (PC 에 transforms3d/numpy2 충돌 사정이 있어 외부 의존을 안 쓴다)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class AngleUnwrapper:
    """-π..π 로 감기는 각도를 연속(누적) 각도로 편다.
    회전 테스트에서 180° 를 넘겨도 총 회전량이 그대로 쌓이게 하기 위함."""
    def __init__(self):
        self.prev = None
        self.total = 0.0     # 첫 샘플 기준 누적 회전량(rad)
        self.first = None

    def update(self, angle: float) -> float:
        if self.prev is None:
            self.prev = angle
            self.first = angle
            return 0.0
        d = angle - self.prev
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        self.total += d
        self.prev = angle
        return self.total


class OdomCheck(Node):
    def __init__(self):
        super().__init__('odom_check')

        # ── 명령(cmd_vel) 적분: 마지막 명령을 벽시계로 적분 ──
        self.cmd_v = 0.0          # 현재 명령 선속도 (m/s)
        self.cmd_w = 0.0          # 현재 명령 각속도 (rad/s)
        self.cmd_dist = 0.0       # 명령 이동거리 누적 (부호 있는 ∫v dt)
        self.cmd_path = 0.0       # 명령 이동거리 누적 (절대값 ∫|v| dt)
        self.cmd_rot = 0.0        # 명령 회전량 누적 (∫w dt)
        self.last_cmd_time = None

        # ── /odom_raw (바퀴) ──
        self.raw_x0 = self.raw_y0 = None    # reset 이후 시작 위치
        self.raw_path = 0.0                 # 경로 길이 ∫|v| dt (twist 로 적분)
        self.raw_x = self.raw_y = 0.0
        self.raw_theta = 0.0
        self.raw_unwrap = AngleUnwrapper()
        self.raw_last_time = None
        self.raw_v = 0.0

        # ── /odom (EKF) ──
        self.ekf_x0 = self.ekf_y0 = None
        self.ekf_x = self.ekf_y = 0.0
        self.ekf_theta = 0.0
        self.ekf_unwrap = AngleUnwrapper()
        self.ekf_seen = False

        # ── /ebimu_data (IMU 절대 yaw) ──
        self.imu_unwrap = AngleUnwrapper()
        self.imu_dyaw = 0.0
        self.imu_yaw = 0.0
        self.imu_seen = False

        self.create_subscription(Twist,    '/cmd_vel',    self.on_cmd,  10)
        self.create_subscription(Odometry, '/odom_raw',   self.on_raw,  10)
        self.create_subscription(Odometry, '/odom',       self.on_ekf,  10)
        self.create_subscription(Imu,      '/ebimu_data', self.on_imu,  50)

        self.create_service(Trigger, '~/reset', self.on_reset)

        # rqt_plot 로 실시간 그래프를 그릴 수 있게 핵심값을 토픽으로도 발행한다.
        #   rqt_plot /odom_check/raw_theta_deg /odom_check/imu_dyaw_deg   ← 회전 겹쳐보기
        #   rqt_plot /odom_check/cmd_path /odom_check/raw_path            ← 직진 겹쳐보기
        self.pub = {n: self.create_publisher(Float64, f'~/{n}', 10) for n in (
            'lin_scale', 'rot_scale', 'cmd_path', 'raw_path',
            'raw_theta_deg', 'imu_dyaw_deg')}

        # 명령 적분용 고속 타이머(20Hz) + 출력용(2Hz)
        self.create_timer(0.05, self.integrate_cmd)
        self.create_timer(0.5, self.print_table)

        self.get_logger().info('odom_check 시작 — 토픽 대기 중. reset: '
                               'ros2 service call /odom_check/reset std_srvs/srv/Trigger')

    # ── 콜백 ────────────────────────────────────────────────────────────
    def on_cmd(self, msg: Twist):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z

    def integrate_cmd(self):
        now = self.get_clock().now()
        if self.last_cmd_time is not None:
            dt = (now - self.last_cmd_time).nanoseconds / 1e9
            self.cmd_dist += self.cmd_v * dt
            self.cmd_path += abs(self.cmd_v) * dt
            self.cmd_rot  += self.cmd_w * dt
        self.last_cmd_time = now

    def on_raw(self, msg: Odometry):
        p = msg.pose.pose.position
        if self.raw_x0 is None:
            self.raw_x0, self.raw_y0 = p.x, p.y
        self.raw_x = p.x - self.raw_x0
        self.raw_y = p.y - self.raw_y0

        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.raw_theta = self.raw_unwrap.update(yaw)

        # 경로 길이는 발행된 선속도(twist.linear.x)를 적분해서 얻는다
        self.raw_v = msg.twist.twist.linear.x
        now = self.get_clock().now()
        if self.raw_last_time is not None:
            dt = (now - self.raw_last_time).nanoseconds / 1e9
            self.raw_path += abs(self.raw_v) * dt
        self.raw_last_time = now

    def on_ekf(self, msg: Odometry):
        self.ekf_seen = True
        p = msg.pose.pose.position
        if self.ekf_x0 is None:
            self.ekf_x0, self.ekf_y0 = p.x, p.y
        self.ekf_x = p.x - self.ekf_x0
        self.ekf_y = p.y - self.ekf_y0
        self.ekf_theta = self.ekf_unwrap.update(
            yaw_from_quaternion(msg.pose.pose.orientation))

    def on_imu(self, msg: Imu):
        self.imu_seen = True
        self.imu_yaw = yaw_from_quaternion(msg.orientation)
        self.imu_dyaw = self.imu_unwrap.update(self.imu_yaw)

    def on_reset(self, request, response):
        self.cmd_dist = self.cmd_path = self.cmd_rot = 0.0
        self.raw_x0 = self.raw_y0 = None
        self.raw_path = 0.0
        self.raw_unwrap = AngleUnwrapper()
        self.ekf_x0 = self.ekf_y0 = None
        self.ekf_unwrap = AngleUnwrapper()
        self.imu_unwrap = AngleUnwrapper()
        response.success = True
        response.message = '누적값 0 으로 초기화'
        self.get_logger().info('── reset ── 누적값 0')
        return response

    # ── 출력 ────────────────────────────────────────────────────────────
    @staticmethod
    def _ratio(num: float, den: float, eps: float = 1e-3):
        if abs(den) < eps:
            return None
        return num / den

    def print_table(self):
        deg = math.degrees
        raw_disp = math.hypot(self.raw_x, self.raw_y)     # reset 이후 직선 변위
        ekf_disp = math.hypot(self.ekf_x, self.ekf_y)

        lin_scale = self._ratio(self.raw_path, self.cmd_path)          # ① 선속도 스케일
        rot_scale = self._ratio(self.raw_theta, self.imu_dyaw)         # ② 회전 스케일

        lin_txt = f'{lin_scale:5.3f}' if lin_scale is not None else '  -  '
        rot_txt = f'{rot_scale:5.3f}' if rot_scale is not None else '  -  '
        ekf_line = (f'net={ekf_disp:6.3f} m  θ={deg(self.ekf_theta):7.2f}°'
                    if self.ekf_seen else '(수신 없음 — EKF 미기동?)')
        imu_line = (f'yaw={deg(self.imu_yaw):7.2f}°  Δyaw={deg(self.imu_dyaw):7.2f}°'
                    if self.imu_seen else '(수신 없음 — IMU 미기동?)')

        print('\n' + '─' * 62)
        print(f'CMD      v={self.cmd_v:+.3f} w={self.cmd_w:+.3f} | '
              f'거리 {self.cmd_dist:+6.3f} m  경로 {self.cmd_path:6.3f} m  회전 {deg(self.cmd_rot):7.2f}°')
        print(f'ODOM_RAW v={self.raw_v:+.3f} | net={raw_disp:6.3f} m  '
              f'경로 {self.raw_path:6.3f} m  θ={deg(self.raw_theta):7.2f}°')
        print(f'EKF/odom {ekf_line}')
        print(f'IMU      {imu_line}')
        print(f'  ① 선속도 스케일 raw_경로/cmd_경로 = {lin_txt}   (1.000 이면 정상)')
        print(f'  ② 회전  스케일 raw_Δθ/imu_Δyaw    = {rot_txt}   (1.000 이면 정상)')

        # rqt_plot 용 토픽 발행 (비율은 아직 못 구하면 0.0 으로)
        self.pub['lin_scale'].publish(Float64(data=lin_scale if lin_scale is not None else 0.0))
        self.pub['rot_scale'].publish(Float64(data=rot_scale if rot_scale is not None else 0.0))
        self.pub['cmd_path'].publish(Float64(data=self.cmd_path))
        self.pub['raw_path'].publish(Float64(data=self.raw_path))
        self.pub['raw_theta_deg'].publish(Float64(data=deg(self.raw_theta)))
        self.pub['imu_dyaw_deg'].publish(Float64(data=deg(self.imu_dyaw)))


def main(args=None):
    rclpy.init(args=args)
    node = OdomCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
