# Relay Robot — 작업 지시

## 0. 시작하기 전에 — 이 순서로 한다

### ① `docs/` 를 먼저 읽는다

```
docs/DEBUG_LOG_<가장 최신 날짜>.md   ← 정본. 0절 요약만 읽어도 현재 상태를 안다
docs/wiki/index.md                   ← 전체 지도
docs/wiki/log.md                     ← 날짜순 이력
```

**날짜가 가장 최신인 DEBUG_LOG 가 정본이다.** 각 문서의 **0절**이 "다음 세션이 이것만
읽으면 되는 요약" 으로 되어 있고, **무엇이 확정(종결)이고 무엇이 미결인지** 명시돼 있다.

> 코드와 git 히스토리만으로는 "왜 이렇게 했는지" 와 "무엇이 **의도적으로** 미해결인지"
> 를 알 수 없다. 이걸 안 읽으면 이미 닫힌 항목을 다시 열게 된다 —
> `DEBUG_LOG_2026-09-03.md` 1절에 그걸로 한 세션을 통째로 날린 기록이 있다.

이전 로그와 충돌하는 내용이 있으면 **날짜가 최신인 쪽이 맞다.**

### ② `git fetch` 로 원격과 비교한다

```bash
git fetch origin && git log --oneline HEAD..origin/main
```

**`git status` 가 clean 한 것은 로컬이 최신이라는 뜻이 아니다.** PC 와 젯슨 두 곳에서
작업하므로 갈라지기 쉽다. 2026-09-22 에 2주 낡은 트리 위에서 하루치를 작업하고 전부
버린 적이 있다 (`DEBUG_LOG_2026-09-22.md` 1절).

---

## 1. ⏳ 지금 막혀 있는 것 — 다음 세션의 첫 작업

> ### ❌ 젯슨에 빌드가 안 돼 있다 (2026-09-22 커밋 `1284559` 기준)
>
> 그날 젯슨 전원이 꺼져 있어서 PC 에서 코드만 고치고 푸시까지만 했다.
>
> ```bash
> ssh robot "cd ~/mobile_robot_proto_type && git pull"
> ssh robot "cd ~/mobile_robot_proto_type && rm -rf build install log && colcon build --symlink-install"
> ```
>
> - **재빌드 필수.** launch 파일과 `config/*.lua` 는 `data_files` 로 설치되므로
>   `--symlink-install` 만으로는 반영되지 않는다. 빌드를 건너뛰면 **예전 launch 가
>   그대로 돌아 IMU/EKF 를 띄우고 TF 가 이중 발행된다.**
> - `rm -rf` 를 붙인 이유: 폐지한 `install/relayrobot_driver/` 가 남아 있으면 낡은
>   사본이 계속 돈다.
>
> **USB 장치도 확인해야 한다** — 09-13 기준 전부 분리 상태였고 전원을 12V 로
> 재구성 중이었다. 12V 에서는 모터 최고속도가 절반 이하가 되는데, **느리다고
> 기구학 상수를 의심하면 안 된다.**

빌드 후 순서와 검사 목록은 `docs/DEBUG_LOG_2026-09-22.md` **6절**에 있다.

---

## 2. 건드리면 안 되는 확정 사항

- **기구학 상수: `wheel_radius=0.0325` / `wheel_base=0.22` / `rpm_scale=600`.**
  2026-09-03 실측 확정. **재측정을 제안하지 말 것.**
- **라이다는 A1/A2 가 아니라 S2 계열이다** (1,000,000 보드). `sllidar_s2_launch.py`.
- **numpy 는 2.2.6. 내리지 말 것.** ROS 라이브러리가 깨지면 그 라이브러리를 올린다.
- **부호는 `MotorDriver` 가 전부 흡수한다** (`DIR_L`/`DIR_R`). 드라이버 밖에서
  다시 뒤집으면 이중 반전이 된다.

## 3. 현재 구성 (2026-09-22 ~)

**IMU 는 선택 사항이다. 기본은 라이다 + 바퀴 오도메트리.**
2D SLAM 에서 yaw 를 잡는 주체는 IMU 가 아니라 라이다 scan matching 이다.

| | `use_imu:=false` (기본) | `use_imu:=true` |
|---|---|---|
| `/odom` 발행 | 드라이버 | `ekf_node` |
| `odom→base_link` TF | **드라이버** | **`ekf_node`** |
| 드라이버 원본 토픽 | `/odom` | `/odom_raw` |

> ⚠️ **`odom→base_link` TF 발행자는 시스템에 하나뿐이어야 한다.** 둘이 되면 TF 가
> 두 값 사이에서 튀고 원인 추적이 매우 어렵다. 발행자가 둘이 되는 흔한 경로:
> `ros2 run relayrobot_driver main_driver` (폐지됨, 젯슨에 잔재 가능),
> `hw_test` GUI 의 EKF Start 버튼.
>
> ⚠️ **`ekf_node` 를 IMU 없이 띄우지 말 것.** `ekf.yaml` 은 yaw 소스가 IMU 단독이라
> **에러 없이 조용히 `/odom` 의 yaw 가 0 에 고정된다.**

**매핑 중 속도 제한 — 취향이 아니라 물리적 한계다.** 라이다가 10Hz(한 스캔 100ms)이고
IMU 가 없으면 스캔 내 모션 왜곡을 보정할 수단이 없다.

- 각속도 **≤ 0.5 rad/s** (1.0 rad/s 면 한 스캔에 5.7° 가 번진다)
- 선속도 **≤ 0.3 m/s**, 제자리 회전 회피
- **teleop 기본값이 그 2배다**: `-p speed:=0.15 -p turn:=0.4` 로 띄울 것

## 4. 저장소 구조

- 활성 파일 지도와 **공부할 때 읽는 순서**는 `README.md` 의 "파일 구조" 절.
- 안 쓰는 파일은 `old_file/` 에 있다 (지운 게 아니다). 사유는 `old_file/README.md`.
- 워크스페이스 패키지는 **5개**.
- ⚠️ `src/ddsm_example/mpc_tubempc/TubeMPCPlanner.py` 는 **옮기면 MPC 가 죽는다** —
  `bridge_node.py` 가 sys.path 로 가져오며 경로가 코드에 박혀 있다.

## 5. 작업 방식

- **토픽 이름을 바꾸면 `grep -rn create_subscription` 을 먼저 돌린다.** ROS 2 는 구독자
  없는 토픽도, 발행자 없는 토픽도 에러를 내지 않는다. "노드는 떠 있는데 화면이 안
  변한다" 로만 드러난다. **문서 안의 `ros2 topic echo` 도 같이 틀려진다.**
- **파일을 옮기면 문서의 명령어 경로도 같이 본다.**
- **같은 내용을 두 문서에 두지 않는다.** 한쪽만 갱신되면 대개 안 고친 쪽을 먼저 읽는다.
- **실기에서 값이 확정되면 그날 저장소에 반영한다.** 미루면 다음 세션이 그 몇 배를 쓴다.
- 작업이 끝나면 `docs/DEBUG_LOG_<오늘 날짜>.md` 를 같은 형식으로 남기고
  `docs/wiki/log.md` 에 항목을 추가한다.
- 프로토콜이 의심되면 문서가 아니라 **펌웨어 소스**(`src/ddsm_example/ddsm_example/`)를 본다.

## 6. 원격 운용

```bash
ssh robot "..."                                  # 젯슨 (Humble)
scripts/robot-up.sh {sensors|full|slam|nav}      # tmux 기동. SSH 끊겨도 산다
ssh robot robot-off                              # ★ 전원 그냥 뽑지 말 것 (SD카드)
```

- 젯슨 = **Ubuntu 22.04 / Humble**, PC = Ubuntu 24.04 / Jazzy. **젯슨은 Jazzy 가 아니다.**
- 제어 루프는 전부 젯슨에서 돈다. PC 는 보고 목표를 주는 쪽이다.
- 자율주행 중 비상정지는 "발행을 멈추는 것" 이 아니다. MPC 를 죽여야 한다:
  `ssh robot 'tmux kill-window -t robot:mpc'`
