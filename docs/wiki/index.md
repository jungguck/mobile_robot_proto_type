# 🗂️ Wiki Index

이 위키는 Relay Robot 프로젝트의 모든 기술적 의사결정과 지식을 체계적으로 저장합니다.

## 🤖 Entities (객체)
- [[Relay_Robot_Hardware]]: 사용된 모터, 센서 사양.
- ⚠️ `ROS2_Packages`: **아직 페이지 없음** (링크만 존재). 패키지별 분석 필요.

## 💡 Concepts (개념)
- [[System_Architecture]]: 전체 데이터 흐름, 레이어 구조, **원격 운영 분담**, 안전 원칙.
- [[TF_Coordinate_System]]: 좌표계 설계와 보정 원리. **프레임을 섞으면 생기는 일**,
  TF 조회에 timeout 을 주면 안 되는 이유, PC↔젯슨 시계 동기.
- [[MPC_Controller]]: Tube-MPC 구조와 파라미터. **QP infeasible 원인**,
  참조 궤적 생성, "경로 없으면 선다" 안전 규칙, 계산 부하 실측.

## 🛠️ Logs (기록)
- [[Debugging_Experience]]: 발생했던 문제와 해결 과정의 집대성.
- [[log]]: 전체 활동 타임라인.

### 세션별 상세 디버그 로그
- `docs/DEBUG_LOG_2026-07-09.md`: 하드웨어 브링업 재검증, EKF NaN 원인, 원격 분기 정리.
- `docs/DEBUG_LOG_2026-08-22.md`: DDSM400 통신 규약 확정(type/단위/부호/ID), IMU 무발행,
  오도메트리 직진 측정, EKF 융합 전략 변경.
- `docs/DEBUG_LOG_2026-09-03.md`: **기구학 상수 확정**(r=0.0325/base=0.22/rpm_scale=600),
  젯슨 부팅 실패(QSPI 펌웨어), 원격 접속 방침.
- `docs/DEBUG_LOG_2026-09-06.md`: **원격 환경 정본.** SSH 키 인증, 워크스페이스 최초 빌드,
  원격 센서 수신 성공(라이다 10Hz/IMU 50Hz), 라이다 S2 정정, numpy 2.2.6 고정.
- `docs/DEBUG_LOG_2026-09-07.md`: PC(Jazzy)↔젯슨(Humble) cross-distro 근거, `ROS_DOMAIN_ID=0` 확정,
  오도메트리 진단 노드 `tools/odom_check.py`, conda 가 rclpy 를 가리는 함정.
- `docs/DEBUG_LOG_2026-09-13.md`: (오전) 재가동 점검 — **진단 사다리**(핑→포트→SSH→ROS→노드→USB),
  12V 전원 재구성과 그 대가, **USB 전 장치 분리 회귀**, SD카드 안전 종료 절차.
  (오후) **자율주행 체인 코드 정비** — tf2 map 프레임 일원화, `/cmd_vel` watchdog,
  A\* 장애물 팽창, `/mpc/*` 원격 관측 토픽. **QP 100% 실패 원인 규명**(heading 제약 0.25rad).
  **로봇 없이 리허설**(`tools/mpc_sim.py`)로 목표 도달까지 검증.
  코드 리뷰에서 **안전 결함 1건**(경로 없으면 장애물로 직진) 발견·수정.

---
*Last updated: 2026-09-13*
