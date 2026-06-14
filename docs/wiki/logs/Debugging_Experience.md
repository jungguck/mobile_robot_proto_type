# 🛠️ Debugging Experience

프로젝트 개발 과정에서 발생한 주요 문제들과 해결책을 집대성합니다. 이 기록은 향후 유사한 문제 발생 시 가이드라인이 됩니다.

## 1. TF 충돌 (Double TF Issue)
- **현상**: RViz에서 로봇이 비정상적으로 떨리며 SLAM이 경로를 소실함.
- **원인**: 모터 드라이버 노드와 EKF 노드가 둘 다 `odom -> base_link` TF를 동시에 발행함.
- **해결**: 드라이버의 TF 발행 기능을 끄고, 센서 데이터만 전달하도록 수정. EKF 노드만 TF를 책임지도록 단일화함.

## 2. 좀비 프로세스 (Zombie Processes)
- **현상**: GUI 종료 후에도 SLAM이나 드라이버 노드가 백그라운드에 남아 시리얼 포트를 점유함.
- **원인**: 부모 프로세스(GUI)만 죽고 자식 프로세스들이 고아가 됨.
- **해결**: Python의 `os.setsid()`를 사용해 새로운 프로세스 그룹을 생성하고, 종료 시 `os.killpg()`로 그룹 전체를 제거함.

## 3. MPC 궤적 추종 실패 (Blind Control)
- **현상**: 로봇이 목표지점과 상관없는 방향으로 움직임.
- **원인**: 제어기가 현재 로봇의 위치(`/odom`)를 피드백 받지 않고 오픈 루프로 동작함.
- **해결**: MPC 로직 내에 `/odom` 구독(Subscriber)을 추가하여 목표치와의 오차를 실시간 계산하도록 수정.

## 4. 시뮬레이션 시간 오차 (Sim Time Issue)
- **현상**: 실제 로봇 가동 시 데이터가 업데이트되지 않거나 노드가 멈춤.
- **원인**: `use_sim_time` 파라미터가 `true`로 설정되어 실제 시스템 시계를 무시함.
- **해결**: 실제 로봇 런칭 파일에서 해당 값을 반드시 `false`로 명시함.

## 5. RPM 변환 계수 오류 (Odom 속도 10배 오차)
- **현상**: `/odom_raw`의 속도가 실제보다 10배 크게 계산됨. EKF가 엉뚱한 위치를 추정하고 SLAM 지도가 왜곡됨.
- **원인**: `real_robot_driver_260519.py`에서 RPM→m/s 변환 시 `/6.0` 사용. DDSM400은 `spd` 피드백이 실제 RPM의 10배이므로 `/600.0`이 맞음. (cmd 100 → 실제 10 RPM → spd 피드백 100)
- **해결**: `vl = (rpm_L / 600.0) * (2 * math.pi * self.wheel_radius)` 로 수정.
- **파일**: `src/relayrobot_description/relayrobot_description/real_robot_driver_260519.py` line 80-81

## 6. 왼쪽 모터 피드백 부호 미반전 (Odom 방향 오류)
- **현상**: 전진 명령 시 로봇이 제자리 회전만 함. vl < 0, vr > 0 으로 선속도가 상쇄되고 각속도가 비정상적으로 큰 값.
- **원인**: 왼쪽 모터는 `drive()`에서 `-rpm_L`로 음수 명령을 보내므로 `spd` 피드백도 음수로 돌아옴. `motor_node_1.py`는 `rpm_L = -rpm_L`로 부호를 반전하는데 `real_robot_driver_260519.py`에는 이 줄이 없었음. `motor_drive_1.py` 83~84번 줄 주석에 명시되어 있었으나 구현이 누락됨.
- **해결**: `rpm_L, rpm_R = self.driver.read_feedback()` 직후에 `rpm_L = -rpm_L` 추가.
- **파일**: `src/relayrobot_description/relayrobot_description/real_robot_driver_260519.py` line 78
