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
- **위키 정비 (SCHEMA Ingest 워크플로우)**: `log.md`/`index.md` 만 고치고 concepts 갱신을 빠뜨렸던 것을 보완.
    - **신규** [[TF_Coordinate_System]] — 링크만 있고 페이지가 없던 고아 링크였다.
      프레임 혼용의 결과, TF 조회 timeout 금지, 시계 동기
    - **신규** [[MPC_Controller]] — 역시 고아 링크였다. QP infeasible 원인, 참조 궤적 생성,
      파라미터 표, 계산 부하 실측
    - **갱신** [[System_Architecture]] — A\*/MPC/watchdog 계층, **원격 운영 분담표**, 안전 원칙 추가.
      데이터 흐름 4·5단계가 **토픽이 아니라 TF 로 위치를 받는다**는 점 명시
    - **갱신** [[Debugging_Experience]] — 13(QP infeasible) / 14(프레임 혼용) /
      15(**경로 없으면 장애물로 직진**) / 16(자율주행 비상정지) 추가.
      **10번(stale 피드백)을 "미해결" → "해결(2026-09-13)"** 로 정정
    - 남은 고아 링크: `ROS2_Packages` (index 에 미작성으로 명시)
- **상세**: `docs/DEBUG_LOG_2026-09-13.md` 11~18절

## [2026-09-16] Hardware | **모터 실구동 성공** — 명령 규약 재확정 / 좌우 ID 확정 / 떨림 원인 규명
- **내용**: 라이다·IMU 없이 **모터만** 연결해 시험. 09-13 에 "모터 구동 미실시" 로 남았던
  항목을 닫았다. **12V 로 실제로 굴러가는 것을 처음 확인**했다.
- **주행 결과**: 전진 50cm / 전진 30cm / 후진 30cm 전부 성공. odom 오차 1% 이내,
  정지 후 잔여속도 0.0 cm/s. 전후진 대칭. (줄자 실측은 미실시 → `wheel_radius` 보정 미정)
- **★ 떨림(뚝뚝 끊김) 원인 = 저속 헌팅**: 정상상태 속도 변동이
  0.05 m/s 에서 **28%**, 0.10 에서 9%, 0.15 에서 6%, 0.20 에서 **2%**.
  **순항 속도를 0.15 m/s 이상으로 잡을 것.**
    - **배제됨**: `act`(0~100 스윕)·재전송 주기(0.02~0.2s 스윕) 둘 다 원인이 아니다. 재시도 금지
    - 측정은 오른쪽 모터 기준 — **왼쪽 확인은 미완**
- **좌우 확정**: `id=1` 오른쪽(전진 음수) / `id=2` 왼쪽(전진 양수). 좌우 균형은
  cmd 294 에서 R 1.010바퀴 / L 0.986바퀴로 **차이 2.4%, 불균형 없음**
    - ID 등록 확인법: `id=3` 에 명령을 보내 **무응답**이면 그 ID 모터가 없는 것.
      펌웨어 `ddsm_ctrl()` 이 ID 를 **검증하지 않고** 그대로 RS485 에 실어 보내기 때문
- **★ 정정 — `type:210` 인과 무효화**: 08-22 의 "`type:210` 을 안 보내면 구동 명령에
  응답조차 안 한다" 가 **재현되지 않았다.** `type:115` 로도 정상 회전했고,
  펌웨어가 성공 시 찍어야 할 `DDSM210` 에코가 한 번도 오지 않았다.
  실제 보드 펌웨어가 저장소 소스와 다를 가능성. **해당 문서들을 수정했다**
- **새로 문서화한 함정**:
    - **`pos` 는 1회전(0~32767)마다 되감긴다.** 누적은 `mil*32768 + pos` 로 구할 것
    - **반이중 RS485** — 여러 모터에 연속 전송 시 10ms 간격 필수. 충돌하면 `{"T":20010,"crc":0}`
    - 젯슨은 **Python 3.10** — f-string 따옴표 중첩은 3.12+ 문법이라 PC 에서 통과해도 죽는다
    - 부팅 직후 **USB 열거가 늦다.** 1분 시점의 장치 목록을 믿지 말 것
- **전원 차단 순서 문서화**: **젯슨 먼저(`robot-off`), 모터 나중.** heartbeat 가 비활성이라
  모터가 도는 중에 젯슨을 끄면 정지 명령을 보낼 수단이 사라진다
- **★ 진입점 정비 — "매번 맨땅에 헤딩" 문제 대응**: 세션마다 상태를 다시 파악하느라
  시간을 쓰는 게 반복돼서, **단일 진입점 `docs/RUNBOOK.md`** 를 만들었다.
  현재 진행 상태 / 다음 할 일 / **증상별 스크립트 선택표** / 시작·종료 절차 /
  "하지 말 것" 목록 / 세션 종료 시 갱신 규칙. README·위키 인덱스에서 링크했다.
    - 오늘 만든 진단 스크립트가 젯슨 `/tmp` 에 있다가 종료와 함께 날아갔다.
      **쓸만한 임시 스크립트는 `tools/` 에 남긴다** 를 규칙으로 문서화
- **작업**:
    - `docs/DEBUG_LOG_2026-09-16.md` 신규
    - **`docs/RUNBOOK.md` 신규 — 세션 진입점**
    - `tools/motor_diag.py` 신규 — 엔코더 누적치로 구동 여부·좌우 균형·속도 산포 판정
    - `tools/drive_straight.py` 신규 — 전/후진 주행 + 떨림 통계 + 보정계수 계산
    - **수정** `README.md` — RUNBOOK 링크, STAGE 0·1 의 `type:210` 주장 제거,
      실패 체크리스트를 실제로 유효한 진단 절차로 교체
    - `src/ddsm_example/ddsm_python/README.md` 신규 — **레거시 폴더임을 명시**.
      `no_terminal.py` 는 **좌우 ID 가 뒤바뀌어** 있고 `wheel_base` 가 1/10,
      `check_feedback.py`·`test_encoder.py` 는 포트가 IMU 로 하드코딩, `enable_test.py` 는
      ESP32 에 raw 패킷을 쏘는 성립 불가 코드, `propose_mpc.py` 는 import 부터 실패
    - **수정** `docs/DEBUG_LOG_2026-08-22.md` — `type:210` 인과 주장 5군데
    - **수정** [[Relay_Robot_Hardware]] — 12V 구동 실측 확인, 저속 헌팅 표,
      전원 차단 순서, `pos` 되감김, RS485 간격, ID 확인법
- **미해결**: 세션 초반의 **무토크 상태**(응답은 오는데 `spd:0`, 개루프에서도 무반응)가
  무엇 때문에 풀렸는지 규명 못 함. 구버전 `motor_drive.py` 실행 후 정상화됐으나,
  그 시점 보드 타입은 이미 210 이었으므로 타입 차이로는 설명되지 않는다
- **상세**: `docs/DEBUG_LOG_2026-09-16.md`

## [2026-11-01] Decision | IMU 를 파이프라인에서 제거 — 바퀴 odom + LiDAR 로 SLAM
- **내용**: 46일 만의 복귀. IMU 를 설치하지 않기로 확정하고 "IMU 없이 SLAM 이 되는가" 를
  먼저 규명했다. 결론은 **된다**. 단, 막고 있던 것은 IMU 자체가 아니라 **EKF 설정**이었다.
- **핵심 규명**:
    - Cartographer 는 원래부터 `TRAJECTORY_BUILDER_2D.use_imu_data = false` — **2D SLAM 은
      IMU 를 안 쓴다.** SLAM 쪽은 손댈 것이 없었다.
    - 진짜 블로커는 `ekf.yaml` 이었다. 08-22 에 [[Debugging_Experience#12]] 로 도입한
      "방향은 IMU yaw, 위치는 바퀴 vx" 전략 때문에 바퀴의 x·y·yaw·v_yaw 가 **전부 false**.
      → IMU 를 빼면 **yaw 관측이 0 개**가 되어 로봇이 회전을 못 느낀다.
    - **해법은 그 전략을 되돌리는 것.** 끈 이유가 "바퀴 yaw ↔ IMU yaw 충돌" 이었으므로
      IMU 가 없으면 끌 이유도 없다. 바퀴의 x·y·yaw·v_yaw 를 다시 켰다.
- **EKF 노드는 남겼다 (다시 헷갈리지 말 것)**: `odom→base_link` TF 를 내는 노드가 그것뿐이다.
  드라이버는 `TransformBroadcaster` 를 **import 만 하고 안 쓴다** — 이중 발행 방지 목적.
  EKF 를 빼면 TF 사슬이 끊겨 Cartographer 가 못 돈다. 센서가 하나라 융합은 없고 패스스루다.
- **작업**:
    - `docs/DEBUG_LOG_2026-11-01.md` 신규
    - **수정** `config/ekf.yaml` — 바퀴 단독 재작성. `imu0` 삭제, `odom0_config` x·y·yaw·v_yaw 켬
    - **수정** `launch/real_robot_260519.launch.py` — `ebimu_publisher` 노드 삭제
    - **수정** `real_robot_driver_260519.py` — 미사용 `Imu` import + 낡은 "IMU 와 융합" 주석
    - **수정** `scripts/99-robot-devices.rules` — `ttyimu` udev 규칙 삭제
    - **수정** `check_devices.sh` — 2 장치(라이다·모터)로 축소
    - **수정** `scripts/robot-up.sh` — tmux `imu` 창 / `/dev/ttyimu` 대기 / `/ebimu_data` 확인 삭제
    - **수정** `tools/odom_check.py` — IMU 구독 및 ② 회전 스케일 삭제
    - **수정** `docs/RUNBOOK.md` · `README.md` — 상태·아키텍처·토픽표·STAGE 표 갱신
- **덤으로 고친 것**: `README.md` Phase 1 표의 `wheel_base = 0.165m` → **`0.22m`**.
  09-03 로그 3절이 이미 틀렸다고 지적해 둔 항목인데 그때 반영되지 않았다.
- **남겨둔 것 (의도적)**: `src/ebimu_pkg/`, `gui_py/hardware_test.py` 의 IMU 탭.
  돌아가는 경로에서 호출되지 않아 해가 없고 IMU 재도입 시 그대로 쓸 수 있다.
  `my_cartographer.lua` 의 `imu_sampling_ratio` 는 **Cartographer 필수 옵션이라 삭제 금지**.
- **대가**: ① yaw 드리프트를 잡아줄 독립 센서 없음 — 바퀴 헛도는 바닥에서 지도 품질 저하
  ② `odom_check.py` 회전 스케일 자동 계산 상실 → `wheel_base` 는 **제자리 90° 수동 회전**으로 보정
  ③ prior 가 바퀴 하나에서만 나오므로 **저속 헌팅이 더 치명적** (순항 0.15 m/s 이상 필수)
- **미해결 — 실기 검증 전무**: 작업 중 젯슨 전원이 꺼져 있었다. PC 에서 YAML 파싱 /
  launch 모듈 로드 / 스크립트 문법까지만 확인했다. **젯슨 재빌드와 udev 재적용이 선행**이며,
  핵심 검증점은 **제자리 회전 시 `/odom` 의 yaw 가 변하는지**다.
- **갱신 필요 (미반영)**: [[System_Architecture]] · [[Relay_Robot_Hardware]] 가 아직 IMU 를
  구성 요소로 기술하고 있다. 다음 세션에서 반영할 것.
- **상세**: `docs/DEBUG_LOG_2026-11-01.md`

## [2026-11-10] Verify | TF 사슬 점검 — 발행자 3개 확인, 라이다 장착 방향은 미검증
- **내용**: "TF 관계가 아예 없는 것은 아닌가" 를 코드로 확인했다. **있다.**
  `map→odom`(cartographer) / `odom→base_link`(EKF) / `base_link→lidar_v1_1`(robot_state_publisher)
  세 발행자가 각자 한 구간씩 맡는다. URDF 에 좌표도 실제로 들어 있다(라이다 = base_link 기준 12.2cm 위).
- **규명**:
    - **라이다 단독(STAGE 3)에는 TF 가 필요 없다.** `sllidar_node` 는 `/scan` 발행만 하고
      TF 를 조회하지 않는다. 단 RViz 로 보려면 변환이 필요하니
      **Fixed Frame 을 `lidar_v1_1`** 로 두면 TF 없이도 보인다.
    - **SLAM 에는 TF 가 필수다.** `tracking_frame = base_link` 라 `/scan`(`lidar_v1_1` 기준)의
      변환이 있어야 하고, `provide_odom_frame = false` 라 `odom→base_link` 는 외부(EKF)가 줘야 한다.
      → [[2026-11-01]] 에 **EKF 를 못 뺀 이유**가 이것이다.
    - **URDF 바퀴 간격 0.165 vs 실측 0.22 는 실기 SLAM 에 영향 없다.** 오도메트리는 URDF 를
      읽지 않고 `motor_drive_1.py` 의 `wheel_base` 파라미터를 쓴다. URDF 바퀴 좌표는
      RViz 그림과 Gazebo 전용 → 09-03 의 "보류" 판단이 옳았다.
    - **함정**: `lookup_transform_timeout_sec = 0.2` 라 scan 시각의 TF 를 0.2초 안에 못 찾으면
      그 scan 을 버린다. **PC↔젯슨 시계 차**와 **기동 순서**(TF 먼저, cartographer 나중)가 중요한 이유다.
      `/scan` 은 나오는데 지도가 안 생기면 라이다보다 **TF·시계를 먼저 본다.**
- **★ 새 미해결**: URDF `Rigid 40` 의 `rpy="0 0 0"` 은 "라이다 0° = 로봇 전진(+x)" 을 전제하는데
  **물리적으로 검증되지 않았다.** 180° 돌아 꽂혀 있으면 지도가 통째로 회전한다. 방향 오차라
  값만 봐서는 안 보인다. STAGE 3 에서 RViz 로 같이 확인할 것 (RUNBOOK 2절에 항목 추가).
- **작업**: `docs/DEBUG_LOG_2026-11-10.md` 신규 / `docs/RUNBOOK.md` 1절 상태표 + 2절 체크 항목 추가
- **상세**: `docs/DEBUG_LOG_2026-11-10.md`
