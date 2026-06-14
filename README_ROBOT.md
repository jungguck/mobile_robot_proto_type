# Relay Robot: ROS2 MPC + SLAM 자율주행

DDSM400 모터 + EBIMU9DOFV5 IMU + RPLidar 기반 차동 구동 모바일 로봇.  
Cartographer SLAM으로 지도 생성 → A* 경로 계획 → Tube-MPC 제어기로 목표 좌표 자율 주행.

---

## 파일 구조 및 관계도

```
src/
├── relayrobot_description/          # 핵심 패키지
│   ├── config/
│   │   ├── ekf.yaml                 # EKF 설정: odom0=/odom_raw, imu0=/ebimu_data
│   │   └── my_cartographer.lua      # Cartographer SLAM 파라미터
│   ├── launch/
│   │   ├── real_robot_260519.launch.py  # 전체 하드웨어 런치 (모터+IMU+LiDAR+EKF)
│   │   └── cartographer.launch.py       # SLAM 런치
│   ├── urdf/relayrobot.xacro        # 로봇 3D 모델 (링크/조인트 정의)
│   └── relayrobot_description/
│       ├── real_robot_driver_260519.py  # 모터 드라이버 + 휠 오도메트리 노드
│       └── motor_drive_1.py             # DDSM400 JSON 시리얼 통신 클래스
│
├── ebimu_pkg/
│   └── ebimu_pkg/ebimu_publisher.py # IMU 드라이버 노드 (10초 캘리브레이션 포함)
│
├── sllidar_ros2/                    # RPLidar 드라이버 (C++)
│
├── mpc_tubempc_bridge/              # MPC 제어 패키지
│   └── src/mpc_tubempc_bridge/
│       ├── bridge_node.py           # Tube-MPC 노드 (/odom → /cmd_vel)
│       └── path_planner.py          # A* 경로 계획 노드 (/map → /global_path)
│
├── ddsm_example/mpc_tubempc/        # MPC 수학 라이브러리 (ROS 무관)
│   ├── TubeMPCPlanner.py            # Tube-MPC 알고리즘 (polytope, cvxpy 필요)
│   └── ReferenceGenerator.py        # 참조 궤적 생성
│
└── gui_py/                          # Tkinter GUI (선택 사항)
    └── gui_py/main.py
```

### 토픽 / TF 흐름

```
[하드웨어]                [드라이버 노드]              [토픽]
/dev/ttyACM0  ──►  real_robot_driver_260519  ──►  /odom_raw  (nav_msgs/Odometry)
                                             ──►  /joint_states
                   sub: /cmd_vel ◄───────────────────────────
/dev/ttyimu   ──►  ebimu_publisher           ──►  /ebimu_data (sensor_msgs/Imu)
/dev/rplidar  ──►  sllidar_node              ──►  /scan      (sensor_msgs/LaserScan)

[센서 융합]
/odom_raw ──┐
            ├──►  ekf_filter_node  ──►  /odom (nav_msgs/Odometry)
/ebimu_data ┘                      ──►  TF: odom → base_link

[SLAM]
/scan + TF(odom→base_link)  ──►  cartographer_node  ──►  /map (OccupancyGrid)
                                                     ──►  TF: map → odom

[경로 계획]
/map + /odom + /mpc_goal  ──►  path_planner (A*)  ──►  /global_path (nav_msgs/Path)

[MPC 제어]
/odom + /global_path  ──►  bridge_node (TubeMPC)  ──►  /cmd_vel (geometry_msgs/Twist)
                           (내부 의존: ddsm_example/mpc_tubempc/TubeMPCPlanner.py)

[TF 트리 전체]
map ──[cartographer]──► odom ──[ekf_node]──► base_link ──[robot_state_publisher]──► lidar_v1_1
                                                                                 ──► left_wheel_v1_1
                                                                                 ──► right_wheel_v1_1
```

### 주요 ROS2 토픽 정리

| 토픽 | 타입 | 발행 노드 |
|------|------|-----------|
| `/odom_raw` | `nav_msgs/Odometry` | real_robot_driver_260519 |
| `/odom` | `nav_msgs/Odometry` | ekf_filter_node (remapped) |
| `/ebimu_data` | `sensor_msgs/Imu` | ebimu_publisher |
| `/scan` | `sensor_msgs/LaserScan` | sllidar_node |
| `/map` | `nav_msgs/OccupancyGrid` | cartographer_node |
| `/mpc_goal` | `geometry_msgs/PoseStamped` | 외부 입력 (ros2 topic pub / GUI) |
| `/global_path` | `nav_msgs/Path` | path_planner |
| `/cmd_vel` | `geometry_msgs/Twist` | bridge_node |

---

## 시스템 요구사항

- OS: Ubuntu 24.04
- ROS: ROS2 Jazzy
- Python: 3.10+
- 필수 Python 패키지: `numpy scipy cvxpy polytope osqp cvxopt`

---

## 하드웨어 포트 매핑

| 장치 | 포트 | 프로토콜 |
|------|------|----------|
| DDSM HAT(B) 모터 컨트롤러 | `/dev/ttyACM0` | USB-CDC, 115200 bps, ESP32 JSON 모드 |
| EBIMU9DOFV5 IMU | `/dev/ttyimu` | UART, 115200 bps |
| RPLidar | `/dev/rplidar` | UART, 1000000 bps (A3) |

---

## 최초 설치 (1회)

```bash
# 1. ROS2 Jazzy 의존 패키지
sudo apt update
sudo apt install -y \
  ros-jazzy-robot-localization \
  ros-jazzy-cartographer-ros \
  ros-jazzy-tf-transformations \
  ros-jazzy-teleop-twist-keyboard

# 2. Python 의존 패키지 (polytope는 LP 솔버 cvxopt 필요)
pip3 install numpy scipy cvxpy polytope osqp cvxopt

# 3. 시리얼 권한 (재로그인 필요)
sudo usermod -aG dialout $USER

# 4. udev 규칙 (IMU, LiDAR 포트 이름 고정)
sudo bash src/relayrobot_description/scripts/setup_udev_rules.sh
# 설정 후 확인
ls -la /dev/rplidar /dev/ttyimu

# 5. 워크스페이스 빌드 (--symlink-install 필수)
# bridge_node.py가 소스 경로 기반 TubeMPCPlanner import를 사용하므로 심링크 빌드 필요
cd ~/mobile_robot_proto_type
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# 6. bashrc 등록
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/mobile_robot_proto_type/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

## 단계별 실행 가이드

> 모든 터미널에서 소싱 필수:  
> `source /opt/ros/jazzy/setup.bash && source ~/mobile_robot_proto_type/install/setup.bash`

---

### STAGE 1: 모터 단독 테스트

**목표:** DDSM400 시리얼 통신 확인, 전진/후진/회전 명령에 실제 응답 확인

```bash
# [터미널 1] 모터+휠오도메트리 노드 실행
ros2 run relayrobot_description real_robot_driver_260519

# [터미널 2] 전진 명령 (0.1 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# 정지
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

# [터미널 3] 오도메트리 확인
ros2 topic echo /odom_raw --field twist.twist.linear
```

**성공 기준:**
- `/odom_raw`의 `twist.linear.x`가 약 `0.1` (±30%)
- 로봇이 실제로 전진

**실패 체크리스트:**
```
[ ] ls -la /dev/ttyACM0          → 없으면 USB 재연결
[ ] groups | grep dialout        → 없으면 usermod 후 재로그인
[ ] ESP32 모드 점퍼 확인          → Arduino 모드면 JSON 무응답
[ ] ros2 run 실행 로그에 "Connected" 출력 확인
```

---

### STAGE 2: IMU 단독 테스트

**목표:** EBIMU 데이터 수신, yaw 방향 정합성 확인

> **주의:** 노드 시작 후 10초간 캘리브레이션이 진행됩니다. 이 시간 동안 로봇을 움직이지 마세요.

```bash
# [터미널 1] IMU 노드
ros2 run ebimu_pkg ebimu_publisher \
  --ros-args -p port:=/dev/ttyimu -p frame_id:=base_link

# [터미널 2] 데이터 확인
ros2 topic hz /ebimu_data          # 목표: 40~60 Hz
ros2 topic echo /ebimu_data --field orientation
```

**성공 기준:**
- 40~60 Hz 발행
- 로봇을 왼쪽(반시계)으로 돌리면 yaw 값 증가

**실패 체크리스트:**
```
[ ] ls -la /dev/ttyimu              → 없으면: sudo ln -s /dev/ttyUSB0 /dev/ttyimu
[ ] 로그에 "Calibration done!" 확인  → 없으면 10초 더 대기
[ ] python3 src/ebimu_pkg/ebimu_pkg/imu_test_1.py  → raw 시리얼 데이터 직접 확인
```

---

### STAGE 3: LiDAR 단독 테스트

**목표:** /scan 발행 확인, RViz에서 장애물 시각화

```bash
# [터미널 1] LiDAR 노드
ros2 run sllidar_ros2 sllidar_node \
  --ros-args \
  -p serial_port:=/dev/rplidar \
  -p serial_baudrate:=1000000 \
  -p frame_id:=lidar_v1_1 \
  -p scan_mode:=DenseBoost

# [터미널 2]
ros2 topic hz /scan                # 목표: 10 Hz 이상

# [터미널 3] RViz
rviz2
# Fixed Frame: lidar_v1_1 / Add: LaserScan → Topic: /scan
```

**성공 기준:**
- 10 Hz 이상 발행
- RViz에서 주변 벽이 점으로 표시됨

**실패 체크리스트:**
```
[ ] ls -la /dev/rplidar            → 없으면 udev 규칙 확인
[ ] LiDAR 모터 회전 확인 (소리)
[ ] scan_mode 오류 시 파라미터 제거하거나 Standard로 변경:
    -p scan_mode:=Standard
[ ] A1 모델은 baudrate=115200 사용
```

---

### STAGE 4: Odometry (EKF 융합) 테스트

**목표:** 바퀴 오도메트리 + IMU → EKF 융합 → /odom 발행, TF 트리 완성

**선행 조건:** STAGE 1, 2, 3 통과

```bash
# [터미널 1] 전체 하드웨어 launch (이 터미널을 이후 단계에서도 유지)
ros2 launch relayrobot_description real_robot_260519.launch.py

# [터미널 2] 확인
ros2 topic list          # /odom, /odom_raw, /ebimu_data, /scan 모두 보여야 함
ros2 topic hz /odom      # 목표: ~30 Hz
ros2 run tf2_ros tf2_echo odom base_link   # TF 출력되면 EKF 정상

# [터미널 3] 직진 1m 후 위치 확인
ros2 topic echo /odom --field pose.pose.position
```

**성공 기준:**
- `/odom` 30 Hz 발행
- `tf2_echo odom base_link`에서 transform 출력됨
- 1m 직진 시 `position.x ≈ 0.9~1.1`

**실패 체크리스트:**
```
[ ] robot_localization 설치: ros2 pkg list | grep robot_localization
[ ] EKF 로그 확인: "ekf_filter_node" 오류 메시지
[ ] /odom_raw 발행 확인 (STAGE 1 선행)
[ ] ekf.yaml 경로: 
    ls $(ros2 pkg prefix relayrobot_description)/share/relayrobot_description/config/ekf.yaml
```

---

### STAGE 5: SLAM (Cartographer) 매핑

**목표:** 지도 생성, TF map→odom 발행 확인

**선행 조건:** STAGE 4의 `real_robot_260519.launch.py`가 계속 실행 중이어야 함

```bash
# [터미널 2] Cartographer 실행 (터미널 1의 launch는 유지)
ros2 launch relayrobot_description cartographer.launch.py

# [터미널 3] 확인
ros2 topic hz /map                          # 1 Hz 내외
ros2 run tf2_ros tf2_echo map odom          # map→odom TF 출력 확인

# [터미널 4] 키보드로 로봇 조종하며 매핑
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# [터미널 5] RViz
rviz2
# Fixed Frame: map / Add: Map(/map), LaserScan(/scan), TF
```

**지도 저장:**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/robot_map
# 결과: ~/robot_map.pgm + ~/robot_map.yaml
```

**성공 기준:**
- `tf2_echo map odom`에서 데이터 출력
- RViz에서 회색 지도가 주행 경로를 따라 생성됨

**실패 체크리스트:**
```
[ ] cartographer_ros 설치: ros2 pkg list | grep cartographer
[ ] my_cartographer.lua 설치 확인:
    ls $(ros2 pkg prefix relayrobot_description)/share/relayrobot_description/config/my_cartographer.lua
[ ] lidar frame_id 확인: ros2 topic echo /scan --field header.frame_id
    → "lidar_v1_1" 이어야 함 (lua의 tracking_frame과 일치 필수)
[ ] STAGE 4 launch가 실행 중인지 확인 (robot_state_publisher가 없으면 TF 오류)
```

---

### STAGE 6: MPC 제어기 + 좌표 입력 자율주행

**목표:** 목표 좌표 입력 → A* 경로 계획 → Tube-MPC 추종 → 자율 주행

**선행 조건:**
- `polytope`, `cvxpy`, `cvxopt` 설치 완료
- STAGE 5 통과 (map 생성 완료)
- `colcon build --symlink-install`로 빌드됨

```bash
# [터미널 1] 하드웨어 launch (유지)
ros2 launch relayrobot_description real_robot_260519.launch.py

# [터미널 2] Cartographer (유지)
ros2 launch relayrobot_description cartographer.launch.py

# [터미널 3] A* 경로 계획 노드
ros2 run mpc_tubempc_bridge mpc_tubempc_path_planner

# [터미널 4] MPC Bridge 노드
ros2 run mpc_tubempc_bridge mpc_tubempc_bridge \
  --ros-args \
  -p use_goal_topic:=true \
  -p use_global_path:=true \
  -p velocity_limit:=0.2 \
  -p omega_limit:=1.0 \
  -p horizon:=6

# [터미널 5] 목표 좌표 발행 (map 좌표 기준, 1~2m 이내 짧은 거리부터 테스트)
ros2 topic pub /mpc_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" \
  --once

# [터미널 6] 모니터링
ros2 topic echo /cmd_vel                    # MPC 출력 확인
ros2 topic echo /global_path --no-arr       # A* 경로 확인
```

**성공 기준:**
- `/global_path` 발행됨 (A* 경로 계산 성공)
- `/cmd_vel`이 0이 아닌 값으로 발행됨
- 로봇이 목표 방향으로 이동 후 정지
- `/odom`의 `position.x/y`가 목표 ±0.3m 이내

**실패 체크리스트:**
```
[ ] ImportError (TubeMPCPlanner) → 반드시 colcon build --symlink-install 로 재빌드
[ ] ImportError (polytope/cvxpy) → pip3 install polytope cvxpy osqp cvxopt
[ ] /global_path 없음 → A* 노드가 /map 수신 대기 중 (Cartographer 실행 확인)
[ ] "Start or goal cell is not free" → 목표가 장애물 위, 다른 좌표 시도
[ ] MPC QP failed → velocity_limit:=0.1, horizon:=4로 줄여서 재시도
[ ] /cmd_vel 발행되나 로봇 미동 → use_goal_topic:=true 파라미터 확인
[ ] 로봇이 목표 반대 방향 → wheel_base 부호 하드웨어 실측 필요
```

---

## 트러블슈팅

### 모터 미응답
```bash
python3 src/relayrobot_driver/relayrobot_driver/motor_test_1.py
# → "Connected" 출력 안 되면 USB, ESP32 모드 점퍼, 전원 순서로 확인
```

### EKF /odom 미발행
```bash
ros2 pkg list | grep robot_localization
# → 없으면: sudo apt install ros-jazzy-robot-localization
```

### Cartographer apt 설치 안 될 때
```bash
# Jazzy에서 공식 패키지 없을 경우 slam_toolbox 대체 사용
sudo apt install ros-jazzy-slam-toolbox
# my_slam_params.yaml의 use_sim_time: false 유지
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=src/relayrobot_description/my_slam_params.yaml
```

### RViz에서 로봇 떨림 (TF 이중 발행)
```bash
ros2 run tf2_ros tf2_monitor
# odom→base_link 발행자가 2개이면 real_robot_driver의 TF 브로드캐스터 비활성화
# (real_robot_driver_260519.py는 이미 비활성화됨)
```

### MPC 발산 / 진동
```bash
# 제약 완화: velocity_limit, omega_limit 줄이기
# horizon 줄이기: -p horizon:=4
# 먼저 1m 이내 짧은 거리로 테스트
```
