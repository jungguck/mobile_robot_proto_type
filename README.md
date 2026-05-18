# Relay Robot Development Notes

> 안되면 당황하지말고 바로 ChatGPT 던지셈

---

# 현재 상태

## Hardware
- 메인 PC 연결 완료
- 모터 드라이버 연결 완료
- LiDAR 연결 완료
- ROS2 통신 확인 완료
- Diff Drive 기반 주행 가능
- Gazebo 시뮬레이션 가능

---

## Simulation

260112_ 시뮬레이션 xacro 파일에 diff_drive plugin 넣음  
→ `/cmd_vel` 속도 명령어를 각 바퀴 회전 속도로 변환함.

---

# 현재 구조 정리

## 주요 패키지
- `src/relayrobot_description/`
  - URDF, launch, config 파일 보관
  - 실제 로봇 실행 `real_robot.launch.py`
  - SLAM 실행 `cartographer.launch.py`
  - EKF 설정 `config/ekf.yaml`
  - Cartographer 설정 `config/my_cartographer.lua`
- `src/relayrobot_driver/`
  - 모터 드라이버 노드, odom + joint_states 발행
  - 실행 가능한 콘솔 스크립트: `main_driver`, `odom_sub`
- `src/sllidar_ros2/`
  - SLAMTEC LiDAR ROS2 드라이버
  - 실행 파일: `sllidar_node`, `sllidar_client`
- `src/ebimu_pkg/`
  - IMU 퍼블리셔 노드 코드
  - 현재 `setup.py`에 콘솔 스크립트가 등록되어야 정상 실행
- `src/aws-robomaker-hospital-world/`
  - Gazebo 시뮬레이션 world 파일
- `src/ddsm_example/`
  - MPC 프로토타입 코드(`src/ddsm_example/ddsm_python/propose_mpc.py`)
  - Tube MPC 알고리즘 폴더(`src/ddsm_example/mpc_tubempc/`)
- `src/mpc_tubempc_bridge/`
  - 실제 ROS2 odom -> 목표 좌표 -> cmd_vel 제어를 연결하는 브리지 노드

## 설치 및 실행 순서
1. ROS2 환경 설정 및 빌드
```bash
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

2. 시뮬레이션 환경
```bash
ros2 launch relayrobot_description gazebo.launch.py
```

3. 실제 로봇 연결 시
```bash
ros2 launch relayrobot_description real_robot.launch.py
```
실제 로봇에서 IMU까지 함께 쓰려면:
```bash
ros2 launch relayrobot_description real_robot.launch.py use_imu:=true
```

## 실제 로봇 + SLAM + RViz 실행 순서
아래 명령은 각각 다른 터미널에서 실행합니다. 각 터미널마다 `source install/setup.bash`를 먼저 실행하세요.

1. ROS2 및 워크스페이스 준비
```bash
source /opt/ros/jazzy/setup.bash  # 현재 환경에서 설치된 ROS2 버전
source install/setup.bash
```

2. 로봇 연결 및 드라이버 실행
```bash
ros2 launch relayrobot_description real_robot.launch.py use_imu:=true
```

3. SLAM으로 맵 생성
```bash
ros2 launch relayrobot_description cartographer.launch.py
```

4. RViz로 맵 보기
```bash
ros2 launch relayrobot_description display.launch.py
```

5. 간단한 버튼 GUI 실행
```bash
ros2 run gui_py gui_py
```

### GUI 사용 안내
- README는 이제 RViz에서 지도를 확인하고, 버튼 GUI로 간단하게 제어하는 흐름에 집중합니다.
- GUI는 간단한 버튼 형태로 동작합니다.
- 전진 / 후진 / 좌 / 우 / 정지 버튼으로 로봇을 제어합니다.
- SLAM, LIDAR, IMU 시작 버튼은 센서 및 SLAM 프로세스를 켭니다.
- 현재 `Start MPC` 버튼은 기본적인 테스트용 고정 명령을 보냅니다.

### IMU + 엔코더 + SLAM 주의
- IMU와 엔코더(odom)는 SLAM에 보조 정보로 들어가지만, 둘 다 데이터가 불안정하면 SLAM 성능이 떨어질 수 있습니다.
- 권장 순서:
  1. 먼저 LIDAR SLAM만 켜서 맵이 잘 그려지는지 확인
  2. 엔코더/odom을 켜서 보조로 추가
  3. IMU를 켜서 추가 보정을 시도
- IMU가 너무 노이즈가 많거나 타임스탬프가 어긋나면, `use_imu_data=false` 상태로만 쓰는 것이 안전합니다.
- odom/IMU 에러가 의심되면 TF 프레임(`odom`, `base_link`, `map`)과 메시지 타임스탬프를 먼저 확인하세요.

### 시스템 노드 관계 (현재 실행 순서)
- `real_robot.launch.py`
  - `relayrobot_driver/main_driver` : 모터 드라이버 + odom/joint_states 발행
  - `sllidar_ros2/sllidar_node` : LiDAR 스캔 발행
  - `ebimu_pkg/ebimu_publisher` : IMU 데이터 발행 (옵션)
  - `relayrobot_driver/odom_sub` : odom 디버그 구독기
  - `robot_localization/ekf_node` : wheel odom + IMU 융합
- `cartographer.launch.py` : Cartographer SLAM
- `display.launch.py` : RViz 실행
- `gui_py` : 버튼형 GUI 실행

### MPC 관련 의존성
- `mpc_tubempc_bridge`는 내부적으로 `ddsm_example/mpc_tubempc/TubeMPCPlanner.py`를 사용합니다.
- 해당 MPC 코드에는 다음 라이브러리가 필요합니다:
  - `numpy`
  - `scipy`
  - `cvxpy`
  - `osqp` (또는 `cvxpy`에서 지원하는 QP solver)
- `mpc_tubempc_bridge` 패키지 메타데이터에는 아직 이 라이브러리들이 명시되어 있지 않습니다.
  따라서 MPC 기능을 쓰려면 별도로 Python 패키지를 설치해 주세요.

### ROS2 버전 권장
- 이 저장소는 ROS2 `ament_python` 패키지 구조와 `launch_ros` API를 사용합니다.
- 현재 환경에서는 `/opt/ros/jazzy`가 설치되어 있으므로, ROS2 `Jazzy`를 사용할 수 있습니다.
- 일반적으로 `Humble` 이상(`Humble`, `Iron`, `Jazzy`) 호환성을 추천합니다.

6. 모터 드라이버 노드 단독 실행
```bash
ros2 run relayrobot_driver main_driver
```

---

# 현재 미완성 / 체크해야 할 점



---

# 앞으로 해야할 작업

## 1. GUI 제작 (`gui_py`)

### 목적
GUI에서 로봇을 간단하게 조작하기 위함.

### 필요한 기능
- 전진 / 후진
- 좌회전 / 우회전
- 정지
- 속도 조절
- SLAM 시작
- SLAM 종료
- MPC 테스트 실행
- 토픽 상태 확인

### 예상 구조
```text
gui_py/
 ├── main.py
 ├── slam_control.py
 ├── teleop_control.py
 ├── mpc_control.py
 └── ui/
