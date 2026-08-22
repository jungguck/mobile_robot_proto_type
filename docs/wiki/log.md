# 📜 Wiki Log

이 파일은 위키에 가해진 모든 변경 사항을 연대순으로 기록합니다.

## [2026-05-20] Initialize | LLM Wiki 패턴 활성화
- **내용**: `firs_you_need_it.md` 기반의 지식 관리 체계 구축.
- **작업**: 
    - `docs/wiki/` 구조 생성.
    - `SCHEMA.md`, `index.md` 초기화.
    - README rewrite 내용을 기반으로 첫 번째 지식 통합(Ingest) 준비.

## [2026-05-20] Ingest | 초기 프로젝트 지식 통합
- **내용**: README 및 코드 분석을 통한 핵심 지식 위키화.
- **작업**:
    - [[System_Architecture]] 페이지 생성: 레이어 구조 및 데이터 흐름 정의.
    - [[Relay_Robot_Hardware]] 페이지 생성: 모터, 센서 사양 기록.
    - [[Debugging_Experience]] 페이지 생성: TF 충돌, 좀비 프로세스 등 해결 사례 정리.

## [2026-06-14] Bugfix | MPC+SLAM 파이프라인 코드 분석 및 버그 수정
- **내용**: Plan/평가 에이전트 기반 전체 파이프라인 분석 후 코드 버그 2건 수정.
- **작업**:
    - [[Debugging_Experience#5]] RPM 변환 계수 `/6.0` → `/600.0` 수정 (`real_robot_driver_260519.py`)
    - [[Debugging_Experience#6]] 왼쪽 모터 피드백 부호 반전 누락 수정 (`rpm_L = -rpm_L` 추가)
    - `my_slam_params.yaml` `use_sim_time: true` → `false` 수정 (dead config 혼란 방지)
    - `README_ROBOT.md` 생성: 파일 관계도, 토픽/TF 흐름, 6단계 실행 가이드
- **미결**: RPLidar 모델별 scan_mode 실측 확인, EKF 공분산 튜닝

## [2026-08-22] Bugfix | DDSM400 통신 규약 확정 + IMU 무발행 해결
- **내용**: 배터리 충전 후 모터 구동 확인을 시도하다 "연결은 되는데 안 도는" 상태의 진짜 원인을 규명.
  명령 단위·부호·ID 매핑을 전부 실측으로 확정했다.
- **작업**:
    - [[Debugging_Experience#7]] 보드 타입 미설정 → `{"T":11002,"type":210}` (전원 사이클당 1회)
    - [[Debugging_Experience#8]] 명령 속도 10배 오류 → `calculate_rpms()` `×60` → `×600`
    - [[Debugging_Experience#9]] 전진이 후진 → `DIR_R=-1` / `DIR_L=+1` 부호 규약 도입
    - [[Debugging_Experience#10]] stale 피드백으로 인한 유령 거리 (50cm 주행이 59.8cm 로 부풀려짐)
    - [[Debugging_Experience#11]] IMU 3필드 출력 → `/ebimu_data` 무발행 해결
    - [[Debugging_Experience#12]] EKF 융합 전략 변경 (방향은 IMU yaw, 위치는 바퀴 vx 적분)
    - [[Relay_Robot_Hardware]] 통신 규약 / 부호 규약 / 기구학 설정값 표 추가
    - `motor_test0822.py`, `odom_calibrate.py` 신규 작성
- **확정**: id=1 오른쪽 / id=2 왼쪽, cmd·spd 단위 = 0.1RPM, 전진 부호 R=음수·L=양수
- **미결**:
    - `wheel_radius` 캘리브레이션 (바퀴 지름 6.5cm vs 역산 7.4cm 불일치 — 재측정 필요)
    - `timer_callback()` 명령 재전송 미적용 (cmd_vel 끊기면 odom 이 유령 거리 누적)
    - heartbeat 워치독 미설정 (노드가 죽어도 로봇이 안 멈춤)
    - IMU + 바퀴 융합 직진 테스트 미실시
    - URDF 바퀴 간격/지름이 실측과 불일치 (CAD 기준값이라 보류)
- **상세**: `docs/DEBUG_LOG_2026-08-22.md`

