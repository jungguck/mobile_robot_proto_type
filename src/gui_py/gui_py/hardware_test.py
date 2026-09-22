#!/usr/bin/env python3
"""
hardware_test.py - 모터 + IMU + LiDAR 통합 하드웨어 테스트 GUI (ROS2 기반)

gui_control.py(스탠드얼론 직접 제어)와 같은 다크 테마/구조를 따르되,
이 프로젝트는 ROS2 기반이므로 모든 통신을 토픽으로 처리한다.

  - 모터  : /cmd_vel 발행 (방향 버튼 + 속도 슬라이더) + odom 구독해 실측 v/ω 표시

[2026-09-22 — IMU/EKF 는 선택이 되었다]
  기본(use_ekf=False)  드라이버가 /odom 과 odom->base_link TF 를 직접 낸다. EKF 불필요.
  use_ekf:=true        드라이버는 /odom_raw 만 내고, EKF 가 /odom 과 TF 를 낸다.

  ★ 이 GUI 로 EKF 를 띄울 때 주의: odom->base_link TF 발행자는 하나뿐이어야 한다.
    기본 구성에서 EKF 를 추가로 띄우면 드라이버와 EKF 가 같은 TF 를 동시에 쏴서
    TF 가 두 값 사이에서 튄다. 그래서 use_ekf=False 면 EKF Start 를 막아둔다.

    ros2 run gui_py hw_test --ros-args -p use_ekf:=true
  - IMU   : /ebimu_data 구독 → roll/pitch/yaw, 각속도, 수신 Hz
  - LiDAR : /scan 구독 → 수신 Hz, 포인트 수, 최소거리, 정면거리

각 드라이버 노드(real_robot_driver_260519 / ebimu_publisher / sllidar_node)는
GUI 버튼으로 직접 Start/Stop 하며, 해당 토픽 수신 여부로 ● 생사 표시를 갱신한다.

[실행]
  ros_setup
  ros2 run gui_py hw_test
"""
import math
import os
import signal
import subprocess
import threading
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan


class ProcessLauncher:
    """드라이버 노드를 별도 프로세스 그룹으로 띄우고 통째로 종료하는 헬퍼."""

    def __init__(self, command, name):
        self.command = command
        self.name = name
        self.process = None

    def start(self):
        if self.process is not None and self.process.poll() is None:
            return False, f'{self.name} is already running'
        try:
            self.process = subprocess.Popen(
                self.command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=os.environ.copy(),
                preexec_fn=os.setsid,  # 프로세스 그룹 생성 → 일괄 종료용
            )
            return True, f'{self.name} started (PID: {self.process.pid})'
        except Exception as e:
            return False, f'Failed to start {self.name}: {e}'

    def stop(self):
        if self.process is None or self.process.poll() is not None:
            return False, f'{self.name} is not running'
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            self.process.wait()
        except Exception as e:
            return False, f'Error stopping {self.name}: {e}'
        self.process = None
        return True, f'{self.name} stopped'

# ── 드라이버 노드 실행 커맨드 (README STAGE 1~3과 동일) ───────────────────
# [2026-09-22 이전]
#   MOTOR_CMD = ['ros2', 'run', 'relayrobot_description', 'real_robot_driver_260519']
#   인자가 없었다. 드라이버 기본값이 /odom_raw + TF 미발행이던 시절의 코드다.
def _motor_cmd(use_ekf: bool):
    """드라이버 실행 커맨드. use_ekf 가 TF 발행 책임을 결정한다."""
    cmd = ['ros2', 'run', 'relayrobot_description', 'real_robot_driver_260519']
    if use_ekf:
        # EKF 가 /odom 과 TF 를 내므로 드라이버는 원본만 내고 TF 는 양보한다
        cmd += ['--ros-args', '-p', 'odom_topic:=odom_raw', '-p', 'publish_tf:=false']
    else:
        # 드라이버가 /odom 과 TF 를 모두 책임진다 (드라이버 기본값과 동일)
        cmd += ['--ros-args', '-p', 'odom_topic:=odom', '-p', 'publish_tf:=true']
    return cmd
IMU_CMD   = ['ros2', 'run', 'ebimu_pkg', 'ebimu_publisher',
             '--ros-args', '-p', 'port:=/dev/ttyimu', '-p', 'frame_id:=base_link']
LIDAR_CMD = ['ros2', 'run', 'sllidar_ros2', 'sllidar_node',
             '--ros-args',
             '-p', 'serial_port:=/dev/rplidar',
             '-p', 'serial_baudrate:=1000000',
             '-p', 'frame_id:=lidar_v1_1',
             '-p', 'scan_mode:=DenseBoost']

# EKF(robot_localization): /odom_raw + /ebimu_data 융합 → /odom 발행.
# (런치 파일과 동일하게 odometry/filtered 를 odom 으로 리맵)
#
# ★ ekf.yaml 은 yaw 소스가 IMU 단독이다. IMU 없이 띄우면 에러 없이 조용히
#   /odom 의 yaw 가 0 에 고정된다. 반드시 IMU 와 같이 쓸 것.
#   또한 publish_tf: true 이므로 드라이버의 publish_tf 와 동시에 켜면 안 된다.
def _ekf_cmd():
    try:
        from ament_index_python.packages import get_package_share_directory
        ekf_yaml = os.path.join(
            get_package_share_directory('relayrobot_description'), 'config', 'ekf.yaml')
    except Exception:
        ekf_yaml = os.path.expanduser(
            '~/mobile_robot_proto_type/src/relayrobot_description/config/ekf.yaml')
    return ['ros2', 'run', 'robot_localization', 'ekf_node',
            '--ros-args', '--params-file', ekf_yaml,
            '-r', 'odometry/filtered:=odom']

REFRESH_MS  = 250    # UI 갱신 주기 (ms) → 4 Hz
ALIVE_SEC   = 1.5    # 마지막 수신이 이 시간 이내면 토픽 '살아있음'으로 간주

# 다크 테마 색상 (gui_control.py와 동일 팔레트)
S = {
    'bg':      '#1e1e2e',
    'fg':      '#cdd6f4',
    'accent':  '#89b4fa',
    'green':   '#a6e3a1',
    'red':     '#f38ba8',
    'yellow':  '#f9e2af',
    'surface': '#313244',
    'font':    ('Consolas', 11),
    'font_b':  ('Consolas', 11, 'bold'),
    'font_h':  ('Consolas', 14, 'bold'),
}


class HardwareTestNode(Node):
    """
    ROS2 노드 + Tkinter GUI 를 한 객체에서 함께 구동한다.

    [스레드 모델]
    - rclpy.spin(self) 는 데몬 스레드에서 돌며 토픽 콜백을 처리한다.
    - 콜백은 '최신 값 저장 + 카운터 증가'만 하고 위젯을 직접 건드리지 않는다.
    - Tkinter 위젯 갱신은 메인 스레드의 주기적 _refresh()(root.after)에서만 한다.
      (Tkinter는 thread-safe 하지 않으므로 위젯은 메인 스레드에서만 만진다.)
    """

    def __init__(self):
        super().__init__('hardware_test_gui')

        # IMU + EKF 구성을 쓸 것인지. 기본은 False (IMU 없는 구성).
        self.declare_parameter('use_ekf', False)
        self.use_ekf = self.get_parameter('use_ekf').value

        # 드라이버 원본 오도메트리의 토픽 이름은 구성에 따라 다르다.
        #   use_ekf=False → 드라이버가 /odom 을 직접 낸다
        #   use_ekf=True  → 드라이버는 /odom_raw, EKF 가 /odom
        self.odom_raw_topic = '/odom_raw' if self.use_ekf else '/odom'
 # 이건 노드이름 

        # ── 최신 센서 값 캐시 ────────────────────────────────────────────
        self.odom = {'v': 0.0, 'w': 0.0}                       # 드라이버 원본 실측 속도
        self.imu  = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,    # /ebimu_data
                     'gyro_z': 0.0, 'acc_x': 0.0, 'acc_y': 0.0}
        self.scan = {'count': 0, 'min': 0.0, 'front': 0.0}     # /scan 요약
        self.odomf = {'x': 0.0, 'y': 0.0, 'yaw': 0.0,          # /odom (위치 추정)
                      'v': 0.0, 'w': 0.0}

        # ── 토픽별 수신 카운터/시각 (Hz 계산 및 생사 판정용) ─────────────
        self._count = {'odom': 0, 'imu': 0, 'scan': 0, 'odomf': 0}
        self._last_recv = {'odom': 0.0, 'imu': 0.0, 'scan': 0.0, 'odomf': 0.0}
        self._last_refresh = time.time()

        # ── ROS 인터페이스 ──────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # [2026-09-22 이전] 이 줄이 '/odom_raw' 로 고정이었다. IMU 없는 구성에서는
        #   그 토픽이 아예 발행되지 않으므로 모터 패널이 영영 '— Hz' 로 남는다.
        #       self.create_subscription(Odometry, '/odom_raw', self._odom_cb, 10)
        self.create_subscription(Odometry, self.odom_raw_topic, self._odom_cb, 10)
        self.create_subscription(Imu, '/ebimu_data', self._imu_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        # /odom 은 구성과 무관하게 "지금 믿을 수 있는 위치 추정" 이다.
        # use_ekf=False 면 위 구독과 같은 토픽이 된다 (같은 토픽 이중 구독은 무해).
        self.create_subscription(Odometry, '/odom', self._odomf_cb, 10)

        # ── 드라이버 노드 런처 ──────────────────────────────────────────
        self.launchers = {
            'motor': ProcessLauncher(_motor_cmd(self.use_ekf), 'Motor'),
            'imu':   ProcessLauncher(IMU_CMD, 'IMU'),
            'lidar': ProcessLauncher(LIDAR_CMD, 'LiDAR'),
            'ekf':   ProcessLauncher(_ekf_cmd(), 'EKF'),
        }

        # ── GUI ─────────────────────────────────────────────────────────
        self.root = tk.Tk()
        self.root.title('Relay Robot - Hardware Test (Motor / IMU / LiDAR)')
        self.root.configure(bg=S['bg'])
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._build_ui()

        # rclpy spin 을 백그라운드 스레드에서 시작
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()

        # 주기적 UI 갱신 시작
        self.root.after(REFRESH_MS, self._refresh)

    # ── ROS 콜백 (값 저장 + 카운트만) ────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.odom['v'] = msg.twist.twist.linear.x
        self.odom['w'] = msg.twist.twist.angular.z
        self._count['odom'] += 1
        self._last_recv['odom'] = time.time()

    def _odomf_cb(self, msg: Odometry):
        # EKF 융합 결과 /odom : 위치(x,y) + 헤딩(yaw) + 속도(v,w)
        self.odomf['x'] = msg.pose.pose.position.x
        self.odomf['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.odomf['yaw'] = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        self.odomf['v'] = msg.twist.twist.linear.x
        self.odomf['w'] = msg.twist.twist.angular.z
        self._count['odomf'] += 1
        self._last_recv['odomf'] = time.time()

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        # 쿼터니언 → 오일러 (deg)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.imu['roll'] = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        sinp = 2 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self.imu['pitch'] = math.degrees(math.asin(sinp))

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.imu['yaw'] = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        self.imu['gyro_z'] = msg.angular_velocity.z
        self.imu['acc_x'] = msg.linear_acceleration.x
        self.imu['acc_y'] = msg.linear_acceleration.y
        self._count['imu'] += 1
        self._last_recv['imu'] = time.time()

    def _scan_cb(self, msg: LaserScan):
        # 유효한(범위 안 + 유한) 거리만 추려서 최소거리 계산
        valid = [r for r in msg.ranges
                 if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        self.scan['count'] = len(valid)
        self.scan['min'] = min(valid) if valid else 0.0

        # 정면(angle 0) 거리: index = (0 - angle_min) / angle_increment
        if msg.angle_increment:
            idx = int(round((0.0 - msg.angle_min) / msg.angle_increment))
            if 0 <= idx < len(msg.ranges):
                r = msg.ranges[idx]
                self.scan['front'] = r if math.isfinite(r) else 0.0
        self._count['scan'] += 1
        self._last_recv['scan'] = time.time()

    # ── UI 구성 ──────────────────────────────────────────────────────────
    def _build_ui(self):
        r = self.root

        # 상단: 타이틀
        top = tk.Frame(r, bg=S['surface'], padx=12, pady=8)
        top.pack(fill='x', padx=10, pady=(10, 0))
        tk.Label(top, text='Hardware Test  ·  Motor / IMU / LiDAR',
                 font=S['font_h'], bg=S['surface'], fg=S['accent']).pack(side='left')

        # ── 드라이버 노드 Start/Stop ─────────────────────────────────────
        drv = tk.LabelFrame(r, text=' 드라이버 노드 ', font=S['font_b'],
                            bg=S['surface'], fg=S['fg'], padx=10, pady=8)
        drv.pack(fill='x', padx=10, pady=8)

        self.dots = {}  # 토픽 생사 표시 ● 레이블
        # use_ekf=False 면 IMU/EKF 를 쓰지 않는 구성이다. 버튼은 남겨두되
        # 라벨로 상태를 드러낸다 (Start 는 _start() 에서 막는다).
        _ekf_label = 'EKF' if self.use_ekf else 'EKF(off)'
        _imu_label = 'IMU' if self.use_ekf else 'IMU(opt)'
        for key, label in [('motor', 'Motor'), ('imu', _imu_label),
                           ('lidar', 'LiDAR'), ('ekf', _ekf_label)]:
            row = tk.Frame(drv, bg=S['surface'])
            row.pack(fill='x', pady=2)

            dot = tk.Label(row, text='●', font=S['font_b'], width=2,
                           bg=S['surface'], fg=S['red'])
            dot.pack(side='left')
            self.dots[key] = dot

            tk.Label(row, text=label, width=9, anchor='w', font=S['font_b'],
                     bg=S['surface'], fg=S['accent']).pack(side='left')

            tk.Button(row, text='Start', font=S['font'], width=7,
                      bg='#45475a', fg=S['green'], relief='flat',
                      command=lambda k=key: self._start(k)).pack(side='left', padx=3)
            tk.Button(row, text='Stop', font=S['font'], width=7,
                      bg='#45475a', fg=S['red'], relief='flat',
                      command=lambda k=key: self._stop(k)).pack(side='left', padx=3)

        # ── 모터 제어 ────────────────────────────────────────────────────
        mot = tk.LabelFrame(r, text=f' 모터  (/cmd_vel → {self.odom_raw_topic}) ', font=S['font_b'],
                           bg=S['surface'], fg=S['fg'], padx=10, pady=8)
        mot.pack(fill='x', padx=10, pady=8)

        # 속도 크기 슬라이더
        sp = tk.Frame(mot, bg=S['surface'])
        sp.pack(fill='x')
        tk.Label(sp, text='Linear (m/s)', width=12, anchor='w', font=S['font'],
                 bg=S['surface'], fg=S['fg']).pack(side='left')
        self.lin_var = tk.DoubleVar(value=0.1)
        tk.Scale(sp, from_=0.0, to=0.3, resolution=0.02, orient='horizontal',
                 variable=self.lin_var, length=180, bg=S['surface'], fg=S['yellow'],
                 troughcolor='#45475a', highlightthickness=0).pack(side='left', padx=6)
        tk.Label(sp, text='Angular (rad/s)', width=14, anchor='w', font=S['font'],
                 bg=S['surface'], fg=S['fg']).pack(side='left')
        self.ang_var = tk.DoubleVar(value=0.4)
        tk.Scale(sp, from_=0.0, to=1.5, resolution=0.1, orient='horizontal',
                 variable=self.ang_var, length=180, bg=S['surface'], fg=S['yellow'],
                 troughcolor='#45475a', highlightthickness=0).pack(side='left', padx=6)

        # 방향 버튼
        btns = tk.Frame(mot, bg=S['surface'])
        btns.pack(pady=6)
        directions = [
            ('▲ 전진', lambda: self._drive(self.lin_var.get(), 0.0), S['green']),
            ('◀ 좌회전', lambda: self._drive(0.0, self.ang_var.get()), S['accent']),
            ('■ 정지', lambda: self._drive(0.0, 0.0), S['red']),
            ('▶ 우회전', lambda: self._drive(0.0, -self.ang_var.get()), S['accent']),
            ('▼ 후진', lambda: self._drive(-self.lin_var.get(), 0.0), S['green']),
        ]
        for text, cmd, color in directions:
            tk.Button(btns, text=text, font=S['font_b'], width=8,
                      bg='#45475a', fg=color, relief='flat', pady=4,
                      command=cmd).pack(side='left', padx=3)

        # 실측 피드백
        self.lbl_motor = tk.Label(mot, text='odom: v=0.000  ω=0.000   (— Hz)',
                                  font=S['font_b'], bg=S['surface'], fg=S['yellow'])
        self.lbl_motor.pack(anchor='w', pady=(4, 0))

        # ── IMU ──────────────────────────────────────────────────────────
        # EKF 2D 오도메트리에 실제로 쓰이는 값: yaw(헤딩), gyro_z(각속도), acc_x/acc_y(가속)
        imu = tk.LabelFrame(r, text=' IMU  (/ebimu_data) ', font=S['font_b'],
                           bg=S['surface'], fg=S['fg'], padx=10, pady=8)
        imu.pack(fill='x', padx=10, pady=8)
        # odom 핵심 값 (강조)
        self.lbl_imu = tk.Label(imu, justify='left', anchor='w', font=S['font_b'],
                                bg=S['surface'], fg=S['yellow'],
                                text='[odom] yaw=—  gyro_z=—\n[odom] acc_x=—  acc_y=—   (— Hz)')
        self.lbl_imu.pack(anchor='w')
        # 참고 값 (roll/pitch)
        self.lbl_imu_ref = tk.Label(imu, justify='left', anchor='w', font=S['font'],
                                    bg=S['surface'], fg=S['fg'],
                                    text='[ref]  roll=—  pitch=—')
        self.lbl_imu_ref.pack(anchor='w')

        # ── LiDAR ────────────────────────────────────────────────────────
        lid = tk.LabelFrame(r, text=' LiDAR  (/scan) ', font=S['font_b'],
                           bg=S['surface'], fg=S['fg'], padx=10, pady=8)
        lid.pack(fill='x', padx=10, pady=8)
        self.lbl_lidar = tk.Label(lid, justify='left', anchor='w', font=S['font_b'],
                                  bg=S['surface'], fg=S['yellow'],
                                  text='points=—  min=—  front=—   (— Hz)')
        self.lbl_lidar.pack(anchor='w')

        # ── Odometry (EKF 융합 /odom) ─────────────────────────────────────
        # SSH 원격 제어 시 로봇이 어디 있는지 확인하는 핵심 값.
        #   use_ekf=False → 드라이버의 바퀴 오도메트리 그대로 (드리프트 누적됨).
        #                   절대 위치는 SLAM 의 map->base_link TF 를 봐야 한다.
        #   use_ekf=True  → 휠 오도메트리 + IMU yaw 를 EKF 가 융합한 값.
        odo_title = (' Odometry  (/odom · EKF 융합) ' if self.use_ekf
                     else ' Odometry  (/odom · 바퀴 적분) ')
        odo = tk.LabelFrame(r, text=odo_title, font=S['font_b'],
                           bg=S['surface'], fg=S['fg'], padx=10, pady=8)
        odo.pack(fill='x', padx=10, pady=8)
        self.lbl_odom = tk.Label(odo, justify='left', anchor='w', font=S['font_b'],
                                 bg=S['surface'], fg=S['green'],
                                 text='pos  x=—  y=—   yaw=—\nvel  v=—  ω=—   (— Hz)')
        self.lbl_odom.pack(anchor='w')
        odo_hint = ('※ EKF Start 필요 (Motor+IMU 먼저 실행)' if self.use_ekf else
                    '※ 바퀴 적분값이라 드리프트가 쌓인다. 절대 위치는 SLAM(map→base_link) 기준')
        tk.Label(odo, text=odo_hint, font=S['font'],
                 bg=S['surface'], fg=S['fg']).pack(anchor='w')

        # ── 하단 상태바 ──────────────────────────────────────────────────
        self.lbl_status = tk.Label(r, text='ready', anchor='w', font=S['font'],
                                   bg=S['surface'], fg=S['fg'], padx=12, pady=6)
        self.lbl_status.pack(fill='x', padx=10, pady=(0, 10))

    # ── 드라이버 Start/Stop ──────────────────────────────────────────────
    def _start(self, key: str):
        # EKF 는 use_ekf=True 일 때만 띄울 수 있다. 기본 구성에서 띄우면
        # 드라이버와 EKF 가 odom->base_link TF 를 동시에 발행해 TF 가 튄다.
        # 게다가 IMU 가 없으면 /odom 의 yaw 가 조용히 0 에 고정된다.
        if key == 'ekf' and not self.use_ekf:
            self._set_status(
                'EKF 사용 안 함 (use_ekf=False). 지금은 드라이버가 /odom 과 TF 를 직접 냅니다. '
                'EKF 를 쓰려면 GUI 를 -p use_ekf:=true 로 다시 실행하세요.')
            return

        ok, msg = self.launchers[key].start()
        # IMU는 시작 직후 바이어스(편차) 제거용 초기 캘리브레이션을 한다.
        # 그동안은 데이터가 발행되지 않으므로 ● 가 초록이 될 때까지 로봇을 정지시킨다.
        if ok and key == 'imu':
            msg += '  —  초기 캘리브레이션 ~10초, 로봇 정지 유지!'
        if ok and key == 'ekf':
            msg += '  —  Motor+IMU 가 먼저 떠 있어야 /odom 발행됨'
        self._set_status(msg)

    def _stop(self, key: str):
        _, msg = self.launchers[key].stop()
        self._set_status(msg)

    # ── 모터 명령 ────────────────────────────────────────────────────────
    def _drive(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)
        self._set_status(f'cmd_vel → v={v:+.2f}  ω={w:+.2f}')

    # ── 주기적 UI 갱신 (메인 스레드) ─────────────────────────────────────
    def _refresh(self):
        now = time.time()
        dt = now - self._last_refresh
        self._last_refresh = now

        # 주기 동안 수신한 메시지 수로 Hz 추정 후 카운터 리셋
        hz = {}
        for k in self._count:
            hz[k] = (self._count[k] / dt) if dt > 0 else 0.0
            self._count[k] = 0

        # 토픽 생사 ● 색상 (마지막 수신 시각 기준)
        # dot 키 → 실제 토픽 키 매핑
        for dot_key, topic_key in (('motor', 'odom'), ('imu', 'imu'),
                                   ('lidar', 'scan'), ('ekf', 'odomf')):
            alive = (now - self._last_recv[topic_key]) < ALIVE_SEC
            self.dots[dot_key].config(fg=S['green'] if alive else S['red'])

        self.lbl_motor.config(
            text=f"odom: v={self.odom['v']:+.3f}  ω={self.odom['w']:+.3f}   "
                 f"({hz['odom']:.0f} Hz)")
        self.lbl_imu.config(
            text=f"[odom] yaw={self.imu['yaw']:+6.1f}°  gyro_z={self.imu['gyro_z']:+6.3f} rad/s\n"
                 f"[odom] acc_x={self.imu['acc_x']:+6.3f}  acc_y={self.imu['acc_y']:+6.3f} m/s²   "
                 f"({hz['imu']:.0f} Hz)")
        self.lbl_imu_ref.config(
            text=f"[ref]  roll={self.imu['roll']:+6.1f}°  pitch={self.imu['pitch']:+6.1f}°")
        self.lbl_lidar.config(
            text=f"points={self.scan['count']}  min={self.scan['min']:.2f} m  "
                 f"front={self.scan['front']:.2f} m   ({hz['scan']:.0f} Hz)")
        self.lbl_odom.config(
            text=f"pos  x={self.odomf['x']:+.3f}  y={self.odomf['y']:+.3f}  "
                 f"yaw={self.odomf['yaw']:+6.1f}°\n"
                 f"vel  v={self.odomf['v']:+.3f}  ω={self.odomf['w']:+.3f}   "
                 f"({hz['odomf']:.0f} Hz)")

        self.root.after(REFRESH_MS, self._refresh)

    def _set_status(self, text: str):
        self.lbl_status.config(text=text)

    # ── 종료 ─────────────────────────────────────────────────────────────
    def shutdown_launchers(self):
        """정지 명령 후 GUI가 띄운 모든 드라이버 노드를 종료한다 (멱등)."""
        try:
            self._drive(0.0, 0.0)
        except Exception:
            pass
        for launcher in self.launchers.values():
            launcher.stop()

    def _on_close(self):
        # 창 X 버튼으로 닫을 때
        self.shutdown_launchers()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareTestNode()

    # 터미널을 닫거나(SIGHUP) 종료 신호(SIGTERM)를 받아도 GUI가 띄운
    # 드라이버 노드들을 함께 종료한다. (드라이버는 setsid 로 분리돼 있어
    # 명시적으로 죽이지 않으면 고아 프로세스로 남는다.)
    def _signal_cleanup(signum, frame):
        node.shutdown_launchers()
        os._exit(0)

    for sig in (signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(sig, _signal_cleanup)
        except Exception:
            pass

    try:
        node.run()
    except KeyboardInterrupt:   # Ctrl-C
        pass
    finally:
        node.shutdown_launchers()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
