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
- **Custom GUI**: 로봇 상태 모니터링 및 수동/반자율 제어 인터페이스.
- **MPC (Model Predictive Control)**: 목표 궤적 추종을 위한 최적화 기반 제어기.

## 🔄 데이터 흐름 (Data Flow)
1. **Raw Data**: 모터 엔코더가 `odom_raw`를 발행.
2. **Fusion**: `robot_localization` 패키지의 `ekf_node`가 IMU와 `odom_raw`를 결합하여 `odom`과 `odom -> base_link` TF 발행.
3. **Correction**: SLAM 노드가 `scan`과 `odom`을 비교하여 `map -> odom` TF 발행.
4. **Control**: GUI/MPC가 `map` 상의 위치를 바탕으로 `cmd_vel` 생성.

## 🔗 관련 문서
- [[TF_Coordinate_System]]
- [[Debugging_Experience]]
