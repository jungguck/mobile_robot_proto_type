# 🤖 Relay Robot Hardware

이 페이지는 프로젝트에 사용된 주요 하드웨어 구성 요소와 사양을 기록합니다.

## DDSM400 모터
- **타입**: Direct Drive Smart Motor (차륜형 로봇용).
- **특징**: 고토크, 정밀 인코더 내장.
- **통신**: Serial 통신을 통해 제어.
- **설정값**:
    - 반지름(Radius): 0.05m (실측값 반영)
    - RPM 보정 계수: 600.0

## RPLidar
- **모델**: RPLidar A1/A2 계열.
- **용도**: 2D 주변 환경 스캔 및 SLAM용 데이터 제공.
- **Topic**: `/scan`
- **Frame ID**: `lidar_v1_1`

## EB-IMU
- **모델**: E2BOX EB-IMU.
- **데이터**: 가속도, 각속도, 절대 방위(Yaw).
- **역할**: EKF 필터의 주요 입력원으로 사용되어 회전 오차 보정.
- **Topic**: `/ebimu_data` (또는 `/imu/data`)

## 제어 PC (Robot PC)
- **OS**: Ubuntu 22.04 LTS
- **ROS Version**: ROS 2 Humble
