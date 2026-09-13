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


## [2026-09-03] Resolve | 기구학 상수 확정 + 젯슨 부팅 문제 + 원격 접속 방침
- **내용**: 8/22 미해결 항목 중 캘리브레이션 관련을 실기 확인으로 종결. Orin Nano 개발자 키트의
  JetPack 6.2.1 부팅 실패 원인(QSPI 펌웨어) 규명. 원격 개발 환경 방침 확정.
- **확정**:
    - `wheel_radius = 0.0325` (지름 65mm) / `wheel_base = 0.22` / `rpm_scale = 600` — **재측정 불필요**
    - 8/22 의 "지름 6.5 vs 역산 7.4cm 불일치" 는 **무효** (근거가 줄자가 아닌 눈대중이었음)
- **작업**:
    - `docs/DEBUG_LOG_2026-09-03.md` 신규 작성 — **현재 상태의 정본**
    - [[Relay_Robot_Hardware]] 기구학 표: "캘리브레이션 미완" → 확정으로 갱신
    - `README.md` STAGE 4: 확정값과 충돌하던 안내(`/300.0`·`/900.0` 조정, `wheel_base 0.165`) 정정
- **미결** (전부 코드 쪽, 소스로 직접 확인):
    - `timer_callback()` 명령 재전송 + `cmd_vel` 타임아웃 → 유령 거리 (**우선순위 1**)
    - `odom.header.stamp` 이중 취득 (`current_time` 재사용하면 됨)
    - `ebimu_publisher.py` `frame_id` 기본값이 URDF 에 없는 `imu_link` (launch 는 덮어쓰므로 잠복)
    - IMU + 바퀴 융합 직진 테스트 미실시 / URDF 실측 불일치 / 라이다 미연결 (8/22 이월)
- **환경**: JetPack 6.2.1 부팅 불가 → **JetPack 5.1.3 을 bridge 로** QSPI 펌웨어 갱신 (재부팅 2회)
- **원격**: VS Code Remote-SSH(작업) / `ssh -X` + `hw_test`(점검) / Foxglove(SLAM 모니터링)
- **프로세스 규칙**: **실기에서 값이 확정되면 그날 저장소에 반영한다.** 미루면 다음 세션이
  이미 닫힌 항목을 다시 열어 시간을 쓴다 (오늘 실제로 발생).
- **상세**: `docs/DEBUG_LOG_2026-09-03.md`

## [2026-09-03] Setup | 젯슨 셋업 가이드 + 목표 환경 확정 (JetPack 6.2.1 / Humble)
- **내용**: 젯슨 준비 절차와 PC 쪽 원격 개발 환경을 한 문서로 정리. 목표 ROS 배포판 확정.
- **확정**: **JetPack 6.2.1 = L4T 36.4.x = Ubuntu 22.04 = ROS 2 Humble** (Jazzy 아님)
    - Humble 은 LTS(2027-05)이고 README 호환표에서 Gazebo 시뮬까지 ✅ → 전 항목 우위
    - JetPack 5.1.x(Ubuntu 20.04)는 Foxy 이고 2023-05 EOL → 목적지가 아니라 펌웨어 경유지
- **정정**: `DEBUG_LOG_2026-09-03.md` 4절에 "JetPack 6 → Jazzy → Gazebo 불가" 로 잘못 적었던
  대목을 바로잡음. JetPack 6 은 24.04 가 아니라 22.04 다.
- **작업**: `docs/JETSON_SETUP.md` 신규 작성
    - 젯슨: 시스템/네트워크/dialout/ROS Humble/프로젝트 의존성/클론·빌드/udev/전원모드·스왑/xauth
    - **ROS apt 서명 키가 2025-06 에 교체됨** → 예전 `apt-key` 방식은 `NO_PUBKEY` 오류.
      `ros2-apt-source` .deb 방식으로 안내
    - PC: SSH 키·config, VS Code Remote-SSH, VcXsrv, Foxglove Studio
    - 원격 코드 편집 4가지 비교 / tmux / 저장소 두 곳 갈라짐 주의

## [2026-09-06] Setup | 원격 개발 환경 완성 — 키 인증 / 최초 빌드 / **원격 센서 수신 성공**
- **내용**: 아침엔 비밀번호 ssh 만 되던 상태에서, 저녁엔 PC 에서 명령 한 줄로 젯슨 노드를 띄우고
  실제 센서 값을 읽는 데까지 갔다. **이 문서가 원격 환경의 정본이다.**
- **성과**:
    - `ssh robot` **키 인증** 완료 — `DEBUG_LOG_2026-09-03` / `REMOTE_ACCESS.md` §3 의 "키 미등록" 은 옛말
    - 젯슨 워크스페이스 **최초 빌드 성공** (`colcon build --symlink-install`, 6/6, 노드 9개 인식)
    - `ssh robot '<명령>'` 한 줄로 ROS 명령 실행 — `~/.bashrc` 를 비대화형 셸에서도 잡히게 수정
    - **센서 실기 검증**: 라이다 `/scan` **10 Hz**(유효 2780점), IMU `/ebimu_data` **50 Hz**
    - `robot-up.sh` — tmux 안 상시 기동. SSH 끊어도 PID 유지 확인
- **정정**:
    - **라이다는 A1 이 아니라 S2 계열**(1,000,000 보드). a1 설정으로는 죽는다
    - 레거시 `relayrobot_driver/main_driver` **제거** — 실드라이버는 `real_robot_driver_260519` 하나
    - **numpy 2.2.6 은 의도된 것.** 1.x 로 내리지 말 것 (tube MPC 의 `cvxpy`/`osqp`/`clarabel` 요구)
- **미결**: **모터 미구동** / EKF·오도메트리 미검증 / `robot-off` 의 전원 차단 미검증
- **상세**: `docs/DEBUG_LOG_2026-09-06.md`

## [2026-09-07] Tooling | PC 를 DDS 노드로 + 오도메트리 진단 노드 `odom_check`
- **내용**: 젯슨이 전원 OFF 인 날이라, **젯슨 없이 되는 것**(PC 준비 + 노드 코드)을 전부 끝냈다.
- **확정**:
    - **PC 에 Humble 을 깔 필요 없다. Jazzy 그대로 젯슨(Humble)과 붙는다.**
      전 토픽이 표준 메시지(`sensor_msgs`/`nav_msgs`/`geometry_msgs`)이고 **커스텀 `.msg` 가 0개** →
      타입 해시 동일 + 양쪽 `rmw_fastrtps_cpp` → cross-distro 통신 성립
    - **`ROS_DOMAIN_ID` 는 0.** `REMOTE_ACCESS.md` §7 의 `30` 은 옛 값 — 달라도 에러 없이 그냥 안 보인다
    - Jazzy 는 `ROS_LOCALHOST_ONLY` 대신 `ROS_AUTOMATIC_DISCOVERY_RANGE`(기본 `SUBNET`) 를 쓴다
- **작업**: `tools/odom_check.py` 신규 — **PC 에서** 도는 오도메트리 진단 노드
    - `/cmd_vel`·`/odom_raw`·`/odom`·`/ebimu_data` 4소스 비교 → ① 선속도 스케일(`rpm_scale`/`wheel_radius`)
      ② **회전 스케일**(`wheel_base`) 검증. `rqt_plot` 용 토픽 6개 발행 + `~/reset` 서비스
    - 설계 근거: **이 IMU 는 자이로를 안 주고 절대 yaw 만 준다** → yaw-rate 교차검증은 불가하지만
      **절대 yaw 를 회전량의 기준자로 삼으면 줄자 없이** 바퀴 회전 오차를 잡는다
- **함정**: **PC 의 conda(파이썬3.13)가 ROS 파이썬(3.12)을 가린다** → `rclpy._rclpy_pybind11` ImportError.
  파이썬 노드 실행 전 `conda deactivate` 필수 (`ros_setup` alias 는 인터프리터까지 안 고쳐준다)
- **미결**: DDS 수신 실증 / 모터 구동 — 둘 다 젯슨이 켜져야 가능
- **상세**: `docs/DEBUG_LOG_2026-09-07.md`

## [2026-09-13] Ops | 재가동 점검 — 원격 연결 재확인 / 12V 전원 재구성 / **USB 분리 확인**
- **내용**: 코드 변경 없음. 하드웨어 재구성(전원 12V) 준비와 원격 연결 재확인.
  모터는 또 못 돌렸다 — **USB 장치가 전부 빠져 있었기 때문**.
- **확인**:
    - 원격 연결 전부 정상 — 핑 ✅ / 포트22 ✅ / `ssh robot` 키 인증 ✅ / 젯슨 `humble`+`DOMAIN=0` ✅
    - **SSH 키는 09-06 부터 이미 등록돼 있었다** (`authorized_keys` 2번째 줄, 지문 일치).
      첫 시도 1회 거부의 원인은 **규명 못 함** — 재발 시 `ssh -v` 로 볼 것
    - **PC IP 도 DHCP 로 바뀐다** — 9/6 `172.30.1.62` → 오늘 `172.30.1.44` (젯슨은 `.45` 유지)
- **방법론 — 진단 사다리 (아래에서 위로)**: `핑 → 포트22 → SSH인증 → 젯슨ROS → 노드 → USB`.
  어느 층에서 끊겼는지가 곧 원인이다. 오늘은 **6층(USB)** 에서 끊겼다
    - 포트 확인은 `nc` 없이 `bash -c 'cat </dev/null >/dev/tcp/IP/22'`
    - SSH 자동화엔 **`BatchMode=yes`** — 비번을 안 물어 프롬프트에서 안 멈춘다
- **오해 교정**:
    - **"PC 에 토픽이 안 보인다" ≠ "DDS 가 안 된다".** `ROS_STATIC_PEERS` 로 docker0 멀티캐스트
      문제를 배제했고, 실제 원인은 **젯슨에 노드 미기동**(부팅만으론 안 뜬다)
    - **`ros2 daemon stop` 을 빠뜨리면** 환경변수를 고쳐도 데몬 캐시 탓에 옛 결과가 나온다
    - **비대화형 셸에선 `conda deactivate` 가 안 먹는다**(훅 미로드) → `export PATH=/usr/bin:...` 로 직접 잡을 것
- **⚠️ 회귀**: `lsusb` 에 **허브 2개 + 블루투스뿐**, `/dev/ttyUSB*` 전무.
  09-06 에 잡혔던 `/dev/motor`·`/dev/rplidar`·`/dev/ttyimu` 가 사라졌다 — 전원 작업 중 물리적 분리.
  **udev 규칙은 남아 있으므로 다시 꽂으면 별칭도 복귀한다.** 현재 유일한 블로커
- **전원 12V 재구성**: 젯슨 DC잭 9–20V 범위라 동작 OK. 모터는 정격 25.2V →
  **최고속도 절반 이하(400→약 190 RPM, 1.36→약 0.65 m/s), 토크는 불변.**
  ⚠️ **느리다고 기구학 상수를 다시 열지 말 것** (09-03 확정값). 모터/젯슨 **전원 분리 유지**
- **프로세스 규칙**: **젯슨 전원을 뽑기 전 반드시 `sudo shutdown -h now`** → 핑 무응답 확인 후 차단.
  SD카드 부팅이라 라이브 차단은 카드 손상. 오늘 2회 모두 준수
- **조작 함정**: `robot` 별칭은 **PC 에만** 있다. 프롬프트 `jk@jk`=PC / `frlab@frlab`=젯슨
- **상세**: `docs/DEBUG_LOG_2026-09-13.md`

## [2026-09-13] Dev | 자율주행 체인 코드 정비 — 프레임 정합 / watchdog / A* 팽창 + **로봇 없이 리허설 통과**
- **내용**: USB 가 빠진 채로 할 수 있는 것을 전부 했다. SLAM→A\*→Tube-MPC 체인의 구멍을 메우고
  **로봇 없이 목표 도달까지 검증**했다.
- **조사 결론**: **없는 것은 코드가 아니었다.** A\*(`path_planner.py`)·Tube-MPC(`bridge_node.py`)·
  Cartographer 런치까지 전부 이미 있었다. **SLAM 실행에 새로 만들 파일은 0개.**
  막혀 있는 건 하드웨어(USB)와 그 아래 단계의 실기 검증뿐
- **고친 것**:
    - **`/cmd_vel` watchdog + 피드백 stale 차단** (09-03 이월 1순위) — DDSM 은 명령 유지형이라
      MPC 가 죽어도 계속 굴러갔고, `read_feedback()` 이 직전 rpm 을 영원히 들고 있어
      **멈춘 로봇이 odom 상 계속 전진**했다. 이제 10Hz 로 재전송 + `cmd_timeout` 0.5s / `feedback_timeout` 0.3s
    - **★ tf2 로 map 프레임 일원화** — `bridge_node` 는 `/odom`(odom 프레임), 경로·목표는 map 프레임인데
      **두 노드 다 tf2 가 없어 변환 없이 뺐다.** SLAM 드리프트 보정분이 그대로 추종 오차로 둔갑하는 구조.
      이제 `global_frame`(기본 map) → `base_link` TF 로 받는다. `global_frame:=odom` 이면 SLAM 없이도 동작
    - **A\* 장애물 팽창 + 미탐색 차단** — 셀 하나만 보던 `is_free()` 가 벽을 긁는 경로를 냈고
      미탐색(-1)을 통행 가능으로 봐 **지도 밖으로 경로를 뚫었다**. EDT 로 지도 수신 시 1회 팽창,
      결과를 `/inflated_map` 으로 발행
    - **`/mpc/*` 원격 관측 토픽** — 제어 루프는 젯슨에서 돌아야 하는데(무선 너머 10Hz = 제어 지터)
      MPC 상태가 `logger.info` 뿐이라 **젯슨 tmux 에 갇혀 있었다**. `reference_path`/`tracking_error`/`status` 신설
    - `robot-up.sh` 에 `slam`/`nav` 모드, PC 용 `config/nav.rviz`, 리허설 노드 `tools/mpc_sim.py` 신규
- **★ 규명 — QP 100% 실패의 원인**: `x_min` 의 θ 한계 `-0.3` 이 tube 타이트닝 후 **실효 0.25 rad(14°)**.
  A\* 경로 첫 참조점의 접선과 로봇 방향은 출발 시 쉽게 20~30° 벌어져 **첫 사이클부터 제약 위반 → 항상 infeasible**.
  오프라인 재현으로 **실패 경계가 정확히 0.25 rad** 임을 측정. `error_yaw_limit` 파라미터로 빼고 기본 3.2.
  **heading 오차는 안전 제약이 아니다** — 지켜야 할 건 위치 오차(tube)
- **QP 부하**: 변수 8개/제약 40행. PC 실측 **평균 4.5ms**(10Hz 예산의 5%) — 젯슨 ARM 에서도 여유.
  **젯슨 실측은 켜지면 할 것**
- **리허설 결과**: 빌드 6/6 · A\* 61점 16ms · 팽창 최소여유 0.250m · 미탐색 누출 0 ·
  목표 (3,2) 도달 **(3.066, 2.004)**, QP 실패 **0회** · **`map→odom` 을 (0.5,0.5,0.3) 틀어놓고도 도달**
- **함정**: `colcon build` 실패 — 6월에 지운 `real_robot.launch.py` 의 **끊긴 심볼릭 링크**가
  `build/`·`install/` 에 잔존. `find build install -xtype l` 로 찾는다
- **미결**: 실기 전부 (USB 재연결 → watchdog 실기 확인 → odom 캘리브 → SLAM). **시계 동기(NTP) 미확인** —
  어긋나면 PC RViz 에서 TF extrapolation 에러가 난다
- **문서 재편**: README 실행 가이드에 **터미널 표기 규칙**(`[PC-n]`/`[젯슨-n]`) 신설 —
  기존 "터미널 1/2/3" 은 **그게 PC 인지 젯슨인지가 문서에 없었다**. 원격 운영이 기본인데 실수를 부른다.
  **진행 순서 표**(단계/터미널/통과 기준/실패 시 복귀)와 `STAGE 0`(USB·전원·**시계 동기**),
  `STAGE 2-B`(**IMU yaw 방향**), `STAGE 4-B`(watchdog), `STAGE 5.5`(리허설) 신설
- **★ IMU yaw 부호**: `ekf.yaml` 이 **방향을 IMU 하나에만** 맡기므로(바퀴 yaw 전부 false)
  **부호가 반대면 EKF 가 진행 방향을 거울로 뒤집어 적분** → 그 `/odom` 이 SLAM prior 로 들어가 지도가 망가진다.
  크기가 아니라 **부호** 문제라 값만 봐선 모른다. `odom_check` 의 **회전 스케일이 음수(≈ −1.0)면 부호 불일치**
- **★ 코드 리뷰에서 안전 결함 1건**: `use_global_path` 모드에서 **경로가 없으면 목표까지 직선으로
  달렸다** — A\* 가 "경로 없음/목표 막힘" 으로 실패한 상황이 곧 그 상황이라, **갈 수 없다고 판정된
  목표를 향해 장애물을 뚫고 직진**하는 구조였다. 이제 경로가 없으면 선다(`NO_PATH`).
  실증: 장애물 한가운데를 목표로 줘도 `cmd_vel=0`, 안 움직임
- **리뷰 반영 (10건 중 8건)**: 낡은 경로 무효화(`path_timeout` 2s + 계획기의 빈 경로 발행) /
  **참조 궤적이 속도한계의 5~7배로 달아나던 것**(셀 스냅 → 세그먼트 보간) /
  **TF lookup timeout 제거**(단일 스레드 spin 에서는 자는 동안 `/tf` 가 안 와 성공 불가) /
  `nearest_free` 가 **벽 건너편으로 시작점을 옮기던 것**(가시선 검사) / A\* 전개 상한 /
  **`nav` 모드 비상정지 안내 정정**(MPC 가 계속 쏘므로 "발행 멈추기" 는 성립 안 함 → `tmux kill-window`)
- **안 고침**: `gui_control.py` 는 Trossen 로봇팔 컨트롤러로 이 저장소와 무관 — **커밋 제외**
- **상세**: `docs/DEBUG_LOG_2026-09-13.md` 11~18절
