# 🏗️ System Architecture

Relay Robot 프로젝트는 센서 융합(Sensor Fusion)과 정밀 제어(MPC)를 결합한 계층적 구조를 가집니다.

## 층위별 구조 (Layered Structure)

### 1. Sensing & Actuation (Hardware)
- **Actuators**: [[Relay_Robot_Hardware#DDSM400-모터]]
- **Sensors**: [[Relay_Robot_Hardware#RPLidar]] (필수),
  [[Relay_Robot_Hardware#EB-IMU]] (**선택** — 2026-09-22 부터 기본 꺼짐)

### 2. Localization & Perception (Estimation)
- **Wheel Odometry (기본)**: 드라이버가 인코더를 적분해 `odom` 과 `odom -> base_link` TF 발행.
  스캔 사이를 메우는 **단기 추정값**이며 절대 정확도를 담당하지 않는다.
- **SLAM (Cartographer)**: Lidar scan matching 으로 자세를 추정하고 지도를 생성.
  누적 오차를 `map -> odom` TF 로 흡수한다. **yaw 의 실질적 주체는 여기다.**
- **EKF (선택)**: `use_imu:=true` 일 때만. `odom_raw`(휠) + IMU 를 융합해 `odom` 과 TF 생성.
  이때 드라이버는 TF 를 내지 않는다. → [[TF_Coordinate_System]]

> IMU 를 빼도 되는 근거와 그때 지켜야 할 속도 제한은 `docs/DEBUG_LOG_2026-09-22.md`.

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
1. **Odometry**: 드라이버가 인코더를 적분해 `odom` 과 `odom -> base_link` TF 발행.
   (`use_imu:=true` 면 드라이버는 `odom_raw` 만 내고, `ekf_node` 가 IMU 와 융합해
   `odom` 과 TF 를 발행한다.)
3. **Correction**: SLAM 노드가 `scan`과 `odom`을 비교하여 `map -> odom` TF 발행.
4. **Planning**: `path_planner` 가 `/map` + `/mpc_goal` + **TF** 로 `/global_path` 생성.
5. **Control**: `bridge_node` 가 `/global_path` + **TF** 로 `/cmd_vel` 생성.

> ★ **4·5 단계는 위치를 토픽이 아니라 TF(`map → base_link`)로 받는다.** `/odom` 은 odom 프레임인데
> 지도·목표·경로는 map 프레임이라, 토픽을 그대로 쓰면 **SLAM 드리프트 보정분이 그대로 추종 오차로
> 둔갑한다.** 자세한 설명은 [[TF_Coordinate_System]].

## 📡 원격 관측 토픽
`/inflated_map` · `/global_path` · `/mpc/reference_path` · `/mpc/tracking_error` · `/mpc/status`
— PC RViz 프리셋: `src/relayrobot_description/config/nav.rviz`

## ⚠️ 운용 제약 — IMU 가 없을 때

라이다(S2 계열)가 10Hz 이고, IMU 가 없으면 스캔 내 모션 왜곡(de-skew)을 보정할
수단이 없다. 한 스캔이 도는 100ms 동안 움직인 만큼 포인트가 번진다.

- 각속도 **≤ 0.5 rad/s** (1.0 rad/s 면 한 스캔에 5.7° 가 번진다)
- 선속도 **≤ 0.3 m/s**
- 제자리 회전 회피 (diff drive 에서 바퀴 슬립이 가장 심한 동작)

`robot-up.sh nav` 가 MPC 에 `omega_limit:=0.5` 를 넘기는 이유가 이것이다.
**teleop 은 기본값이 `turn=1.0` 이라 따로 낮춰야 한다** (`-p turn:=0.4`).

## ⚠️ 안전 원칙
- **경로가 없으면 선다.** A\* 실패·계획기 사망 시 직선으로 달리지 않는다 (`NO_PATH`).
- **자율주행 중 비상정지는 "발행을 멈추는 것" 이 아니다.** MPC 가 10Hz 로 계속 쏘므로
  MPC 를 죽여야 한다: `ssh robot 'tmux kill-window -t robot:mpc'`.
- 미탐색(-1) 영역은 통행 불가로 취급한다(`allow_unknown` 기본 false).

## 🔗 관련 문서
- [[TF_Coordinate_System]]
- [[MPC_Controller]]
- [[Debugging_Experience]]
