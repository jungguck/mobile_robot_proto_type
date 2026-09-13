# 🏗️ System Architecture

Relay Robot 프로젝트는 센서 융합(Sensor Fusion)과 정밀 제어(MPC)를 결합한 계층적 구조를 가집니다.

## 층위별 구조 (Layered Structure)

### 1. Sensing & Actuation (Hardware)
- **Actuators**: [[Relay_Robot_Hardware#DDSM400-모터]]
- **Sensors**: [[Relay_Robot_Hardware#RPLidar]], [[Relay_Robot_Hardware#EB-IMU]]

### 2. Localization & Perception (Estimation)
- **EKF (Extended Kalman Filter)**: `odom_raw`(휠 인코더)와 IMU 데이터를 융합하여 정밀한 `odom` 생성.
- **SLAM (Cartographer)**: Lidar 데이터를 이용해 지도를 생성하고, `map -> odom` TF를 통해 누적 오차 보정.

### 3. Navigation & Control
- **A\* Global Planner** (`path_planner.py`): `/map` 을 **로봇 반경만큼 팽창**시킨 뒤 격자 탐색.
  결과를 `/global_path`, 팽창 지도를 `/inflated_map` 으로 발행.
- **Tube-MPC** ([[MPC_Controller]]): `/global_path` 추종. `/cmd_vel` 생성.
- **Safety (드라이버 계층)**: `/cmd_vel` watchdog — 명령이 `cmd_timeout`(0.5s) 끊기면 모터 정지.
  피드백이 `feedback_timeout`(0.3s) 끊기면 속도를 0 으로 간주(유령 거리 차단).

### 4. Remote Operation (원격 운영)
**제어 루프는 전부 로봇(젯슨)에서 돈다.** 10Hz 루프를 무선 너머에 두면 통신 끊김이
곧 제어 지터가 되기 때문이다. PC 는 **관찰하고 목표를 주는 쪽**이다.

| | 젯슨 (Humble) | PC (Jazzy) |
|---|---|---|
| 드라이버 · EKF · SLAM · A\* · MPC | ✅ | — |
| RViz · rqt_plot · 진단 도구 | ❌ | ✅ |

기동: `robot-up.sh {sensors|full|slam|nav}` (tmux).
cross-distro(PC Jazzy ↔ 젯슨 Humble) 통신 근거는 `docs/DEBUG_LOG_2026-09-07.md`,
접속 방법은 `docs/REMOTE_ACCESS.md`.

## 🔄 데이터 흐름 (Data Flow)
1. **Raw Data**: 모터 엔코더가 `odom_raw`를 발행.
2. **Fusion**: `robot_localization` 패키지의 `ekf_node`가 IMU와 `odom_raw`를 결합하여 `odom`과 `odom -> base_link` TF 발행.
3. **Correction**: SLAM 노드가 `scan`과 `odom`을 비교하여 `map -> odom` TF 발행.
4. **Planning**: `path_planner` 가 `/map` + `/mpc_goal` + **TF** 로 `/global_path` 생성.
5. **Control**: `bridge_node` 가 `/global_path` + **TF** 로 `/cmd_vel` 생성.

> ★ **4·5 단계는 위치를 토픽이 아니라 TF(`map → base_link`)로 받는다.** `/odom` 은 odom 프레임인데
> 지도·목표·경로는 map 프레임이라, 토픽을 그대로 쓰면 **SLAM 드리프트 보정분이 그대로 추종 오차로
> 둔갑한다.** 자세한 설명은 [[TF_Coordinate_System]].

## 📡 원격 관측 토픽
`/inflated_map` · `/global_path` · `/mpc/reference_path` · `/mpc/tracking_error` · `/mpc/status`
— PC RViz 프리셋: `src/relayrobot_description/config/nav.rviz`

## ⚠️ 안전 원칙
- **경로가 없으면 선다.** A\* 실패·계획기 사망 시 직선으로 달리지 않는다 (`NO_PATH`).
- **자율주행 중 비상정지는 "발행을 멈추는 것" 이 아니다.** MPC 가 10Hz 로 계속 쏘므로
  MPC 를 죽여야 한다: `ssh robot 'tmux kill-window -t robot:mpc'`.
- 미탐색(-1) 영역은 통행 불가로 취급한다(`allow_unknown` 기본 false).

## 🔗 관련 문서
- [[TF_Coordinate_System]]
- [[MPC_Controller]]
- [[Debugging_Experience]]
