# 🧹 정리 대상 목록 (2026-09-16 조사)

**아직 아무것도 지우지 않았다.** 조사만 한 상태다. 내일 위에서부터 처리하면 된다.
각 항목에 **왜 지워도 되는지 근거**를 붙였다. 근거가 약한 건 "판단 필요" 로 분류했다.

> ⚠️ **먼저 읽을 것:** 아래 C-1 은 **절대 지우면 안 되는데 지우기 쉬워 보이는** 것이다.

> ### 🎯 그런데 이 정리는 곁가지다
> 최종 목표는 **SLAM → A\* → Tube-MPC 자율주행**이고, 지금 그걸 막고 있는 건
> **라이다·IMU 가 분리돼 있다는 것** 하나뿐이다. 저장소 정리는 SLAM 을 앞당기지 않는다.
> **센서를 다시 꽂고 STAGE 2·3 을 먼저 통과시키는 게 우선이다.**
> 정리는 그게 막혔거나 시간이 남을 때 한다. — `docs/RUNBOOK.md`

---

## A. 지워도 안전 — 참조 0곳, 확인 완료

| # | 파일 | 근거 | git |
|---|---|---|---|
| A-1 | `src/ddsm_example/ddsm_example_downloader.zip` | **22MB.** 벤더 배포 zip. 내용물은 이미 풀려서 `ddsm_example/`·`ddsm_python/` 에 있다. **`.git` 이 86MB 인 주원인** | 추적됨 |
| A-2 | `ROS2_SI.png` | 756KB. **참조 0곳** | 추적됨 |
| A-3 | `my_second_map_r.xcf` | GIMP 원본. **참조 0곳** | 추적됨 |
| A-4 | `frames_2026-01-13_17.35.48.gv` / `.pdf` | 1월 TF 덤프. **참조 0곳.** 필요하면 `ros2 run tf2_tools view_frames` 로 언제든 재생성 | 추적됨 |
| A-5 | `frames_2026-01-27_14.03.31.gv` / `.pdf` | 위와 같음 | 추적됨 |
| A-6 | `gui_control.py` | **Trossen 로봇팔(WXAI) Tkinter 컨트롤러.** `import trossen_arm`, `ROBOT_IP=192.168.1.101` — 이 저장소와 무관. 09-13 에 이미 "커밋 제외" 로 판단됨 | **미추적** |

> A-6 주의: `src/gui_py/gui_py/hardware_test.py` 가 주석에서 "gui_control.py 와 같은 팔레트" 라고
> **언급만** 한다. import 가 아니므로 지워도 동작에 영향 없다.

```bash
git rm src/ddsm_example/ddsm_example_downloader.zip ROS2_SI.png my_second_map_r.xcf \
       frames_2026-01-13_17.35.48.gv frames_2026-01-13_17.35.48.pdf \
       frames_2026-01-27_14.03.31.gv frames_2026-01-27_14.03.31.pdf
rm -f gui_control.py
```

> ℹ️ `git rm` 은 **작업트리에서만** 없앤다. 22MB zip 은 **히스토리에 그대로 남아** `.git` 크기는
> 안 줄어든다. 진짜로 줄이려면 `git filter-repo` 로 히스토리를 다시 써야 하는데,
> **혼자 쓰는 저장소가 아니면 권하지 않는다.** 일단은 앞으로 안 커지게만 막는 걸로 충분하다.

---

## B. 레거시 스크립트 — `src/ddsm_example/ddsm_python/`

폴더 전체가 초기 실험 흔적이다. 자세한 내용은 같은 폴더의 `README.md` 에 정리해 뒀다.

### B-1. 지울 것 (틀렸거나 실행 불가)

| 파일 | 근거 |
|---|---|
| `no_terminal.py` | **좌우 ID 가 뒤바뀜**(`L=1,R=2`), `WHEEL_BASE=0.02`(실측 0.22 의 **1/10**), `WHEEL_RADIUS=0.07`. 참고하면 사고 난다 |
| `enable_test.py` | 38400 보드레이트로 **raw DDSM 패킷을 ESP32 USB CDC 에** 쓴다. 그 포트는 JSON 을 받는다. **성립 자체가 안 됨** |
| `propose_mpc.py` | 존재하지 않는 `motor_driver` 모듈 import. **실행 불가** |
| `check_feedback.py` | 포트가 `/dev/ttyUSB0` = **IMU** 하드코딩. 초기화도 틀린 `{"T":11002,"id":1}` 형태 |
| `motor_node.py`, `motor_node_1.py` | 위 틀린 드라이버들을 쓰는 레거시 ROS 노드 |
| **`motor_drive_1.py`** | ⚠️ **정본과 파일명이 같아서 가장 위험하다.** 정본은 `relayrobot_description/relayrobot_description/motor_drive_1.py` |
| `motor_drive.py` | 초기화 `{"T":11002,"id":N}`, `×60`(정본 ×600), 옛 기구학 상수, 부호 반대 |

> `motor_drive.py` 만은 **2026-09-16 에 무토크 상태를 푼 스크립트**라 이력상 의미가 있다.
> 그 경위는 `DEBUG_LOG_2026-09-16.md` 8절에 기록해 뒀으므로 파일 자체는 지워도 된다.

> `test_encoder.py` 는 지우기 전에 주석의 **`total_ticks = laps*32768 + raw_pos`** 근거가
> 이미 `tools/motor_diag.py` 와 09-16 로그에 옮겨져 있는지 확인할 것. (옮겨져 있다 → 지워도 됨)

### B-2. 남길 것

| 파일 | 이유 |
|---|---|
| `simply.py` | 대화형 JSON REPL. 손으로 명령 쳐볼 때 유용 |
| `serial_simple_ctrl.py` | 위와 비슷 |
| `requirement.txt` | `pyserial==3.5` |
| `README.md` | 이 폴더의 주의사항 정본 |

---

## C. ⚠️ 지우면 안 되는 것 (지우기 쉬워 보이지만)

| # | 대상 | 이유 |
|---|---|---|
| **C-1** | **`src/ddsm_example/mpc_tubempc/`** | **Tube-MPC 의 핵심 모듈이 여기 산다.** `bridge_node.py:19` 가 `this_dir.parents[2]/'ddsm_example'/'mpc_tubempc'` 로 **상대경로를 파고들어** `TubeMPCPlanner` 를 import 한다. 벤더 폴더처럼 보이지만 **load-bearing** |
| C-2 | `src/ddsm_example/ddsm_example/` | **펌웨어 소스 = 프로토콜 정본.** 문서보다 정확하다. `json_cmd.h`, `ddsm_example.ino` |

> **C-1 은 구조 자체가 위험하다.** 핵심 제어 모듈이 "예제 폴더" 안에 있고, 상대경로로
> 불려 온다. 언젠가 `mpc_tubempc_bridge` 안으로 옮기는 게 맞다 — 다만 그건 정리가 아니라
> **리팩터링**이라 따로 시간을 잡아야 한다. 옮길 때 `--symlink-install` 빌드 전제도 같이 본다.

---

## D. 판단 필요 — 지우기보다 "합치기"

| # | 대상 | 상황 | 제안 |
|---|---|---|---|
| D-1 | `README_ROBOT.md` (421줄) | `README.md`(1316줄)와 **내용이 겹친다.** 파일 관계도·토픽/TF 흐름·STAGE 가이드가 양쪽에 있고, README.md 쪽이 상위집합이며 최신이다. 참조는 `wiki/log.md` 의 생성 이력 1곳뿐 | **README.md 에 없는 내용만 흡수 후 삭제.** 두 벌을 유지하면 한쪽만 갱신되어 또 헷갈린다 |
| D-2 | `read_prompt.md` + `GEMINI.md` | README 작성 지시용 프롬프트. `GEMINI.md` 가 `read_prompt.md` 를 참조 | 둘을 하나로 합치거나, 쓰는 도구가 정해졌으면 **그 도구 규약 파일로 통일** |
| D-3 | `first_you_need_it.md` | LLM Wiki 패턴 **원문 복사본.** 이 패턴으로 `docs/wiki/` 가 이미 구축 완료됨 | **역할이 끝났다.** 삭제 후보 |
| D-4 | `FABLE_REASONING.md` | 다른 모델 세션에 붙여넣을 작업 원칙 | 쓰고 있으면 유지. 안 쓰면 삭제 |
| D-5 | `src/relayrobot_driver/` | `setup.py` 의 entry_points 에 **`odom_sub` 하나만** 남았다. `motor_test_1.py` 는 참조 1곳, `motor_id_check.py` 는 5곳(대부분 문서) | `odom_subscriber.py` 만 살리고 나머지 정리. 패키지째 없애려면 `odom_sub` 를 `relayrobot_description` 으로 옮길 것 |
| D-6 | `src/relayrobot_description/launch/gazebo.launch.py` | **Jazzy(PC)에서는 못 쓴다** — Gazebo Classic EOL. Humble/Foxy 에서는 동작 | 시뮬을 안 쓸 거면 삭제. 나중에 쓸 거면 유지 (README 에 이미 경고 있음) |

---

## E. 정리 아님 — 그냥 지워도 되는 빌드 산출물

```bash
rm -rf build install log        # 25MB. .gitignore 에 이미 있어 git 과 무관
colcon build --symlink-install  # 필요하면 언제든 재생성
```

---

## 권장 순서

1. **A 전부** — 근거 확실, 위험 0
2. **B-1** — `ddsm_python/README.md` 는 남길 것
3. **D-1 (`README_ROBOT.md`)** — 중복 문서가 헷갈림의 큰 원인이다
4. **D-3, D-4** — 판단만 하면 됨
5. **D-5, D-6** — 시간 있을 때
6. **C-1 리팩터링** — 별도 작업으로

> 각 단계 뒤에 `colcon build` 가 통과하는지 확인할 것. 특히 B·C·D-5 를 건드린 뒤에는 반드시.
