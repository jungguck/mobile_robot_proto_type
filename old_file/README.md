# 📦 old_file — 현재 체인에서 안 쓰는 파일 보관소

2026-09-22 에 정리했다. **지운 게 아니라 옮긴 것**이고, 원래 경로를 그대로 유지하므로
`old_file/` 만 떼면 예전 위치가 된다.

```bash
# 되돌리기 (예)
git mv old_file/src/ddsm_example/ddsm_python/simply.py src/ddsm_example/ddsm_python/simply.py
```

`COLCON_IGNORE` 파일이 있어서 **colcon 이 이 폴더를 쳐다보지 않는다.** 빌드에 영향 없다.

> **왜 옮겼나:** 모터 관련 코드 사본이 여러 벌 있어서 "지금 실제로 도는 게 어느 것인지"
> 가 헷갈렸다. 공부할 때 읽어야 할 파일만 `src/` 에 남긴다.
> 지금 살아있는 파일 목록은 `README_ROBOT.md` 의 **파일 구조** 절에 있다.

---

## 무엇이 왜 여기 있나

### `src/ddsm_example/ddsm_python/` — 벤더 예제 + 옛 모터 코드 사본

DDSM 보드 제조사 예제와, 거기서 갈라져 나온 옛 드라이버 사본들이다.
**실제로 도는 모터 드라이버는 `src/relayrobot_description/relayrobot_description/motor_drive_1.py` 하나다.**

| 파일 | 비고 |
|---|---|
| `motor_drive.py`, `motor_drive_1.py` | 현재 드라이버의 조상. 기구학 상수가 낡았다 |
| **`motor_node_1.py`** | ⚠️ `odom` + `odom→base_link` TF 를 **직접** 쏜다. 지금 드라이버도 TF 를 내므로 **동시에 띄우면 TF 가 이중 발행된다.** `r=0.05`/`base=0.165` 로 상수도 틀렸다 |
| `motor_node.py` | 위의 이전 판 |
| `simply.py` | 벤더 원본. **모터가 응답하는지만 보고 싶을 때 여전히 쓸모 있다** (`DEBUG_LOG_2026-09-03.md` 9절) |
| `check_feedback.py` | ⚠️ 포트가 `/dev/ttyUSB0`(= IMU) 로 하드코딩돼 있다. 그대로 쓰지 말 것 |
| `enable_test.py`, `no_terminal.py`, `serial_simple_ctrl.py`, `test_encoder.py`, `propose_mpc.py` | 일회성 실험 |

> **펌웨어 소스(`src/ddsm_example/ddsm_example/*.ino`, `json_cmd.h`)는 옮기지 않았다.**
> "프로토콜이 의심되면 문서가 아니라 펌웨어 소스를 봐라" 가 이 프로젝트 규칙이다
> (`DEBUG_LOG_2026-08-22.md` 2절).

### `src/ddsm_example/mpc_tubempc/` — 죽은 코드 2개

| 파일 | 왜 |
|---|---|
| `main_loop.py` | `from communication import CommunicationHandler` — **그런 모듈이 저장소에 없다.** 지금 실행하면 ImportError |
| `ReferenceGenerator.py` | `main_loop.py` 에서만 쓰였다 |

> ⚠️ **`TubeMPCPlanner.py` 는 옮기지 않았다. 옮기면 MPC 가 죽는다.**
> `bridge_node.py` 가 sys.path 에 `src/ddsm_example/mpc_tubempc` 를 넣고
> `from TubeMPCPlanner import TubeMPCPlanner` 로 가져온다. 경로가 코드에 박혀 있다.

### `src/relayrobot_driver/` — 패키지 통째로 (2026-09-22)

`odom_sub`(`odom_subscriber.py`)가 `relayrobot_description` 의 `odom_listener` 와
**완전히 같은 일**을 했다 — 둘 다 `/odom` 을 구독해 x, y, yaw 를 출력. 패키지 하나가
그 중복 노드 하나 때문에 존재하고 있었다.

`odom_listener` 로 합치면서 이쪽의 장점 두 개를 가져갔다:
- 출력을 `print` 가 아니라 `get_logger()` 로 (ROS 로그에 남고 시각이 찍힌다)
- `destroy_node()` 를 `finally` 안으로 (예외로 빠져나갈 때도 정리된다)

`main_driver` 는 2026-09-06 에 이미 엔트리에서 제거된 상태였다.

> 이 패키지의 `package.xml` 이 여기 남아 있지만 **`COLCON_IGNORE` 때문에 빌드되지
> 않는다.** 워크스페이스 패키지는 6개 → **5개**가 됐다.
>
> ⚠️ **젯슨에서는 `install/relayrobot_driver/` 가 남아 있을 수 있다.** 그러면
> `ros2 run relayrobot_driver odom_sub` 가 낡은 사본으로 계속 돈다.
> 깔끔하게 하려면 pull 후 `rm -rf build install log` 하고 다시 빌드할 것.

### 일회성 테스트 노드

| 파일 | 대체재 |
|---|---|
| `src/ebimu_pkg/ebimu_pkg/imu_test_1.py`, `imu_test_2.py` | IMU raw 시리얼 확인용. IMU 는 이제 선택 사항이다 |
| `src/relayrobot_driver/relayrobot_driver/motor_id_check.py` | 모터 ID↔위치는 확정됨 (id=1 오른쪽 / id=2 왼쪽) |
| `src/relayrobot_driver/relayrobot_driver/motor_test_1.py` | `gui_py` 의 `hw_test` 가 같은 일을 더 낫게 한다 |

### 현재 체인에서 안 쓰는 launch / config

| 파일 | 왜 |
|---|---|
| `launch/display.launch.py` + `config/display.rviz` | URDF 만 RViz 로 보는 용도. 실주행 경로가 아니다 |
| `launch/gazebo.launch.py` | 시뮬레이션. 지금은 실기로 간다 |
| `launch/odom.launch.py` | `odom_listener` 하나만 띄운다. `ros2 run` 으로 충분 |
| `my_slam_params.yaml` | 파일 첫 줄에 **"미사용 파일"** 이라고 본인이 적어놨다. slam_toolbox 로 갈아탈 때 참고용 |

### 루트 산출물 / 1회성 스크립트

| 파일 | 왜 |
|---|---|
| `motor_test0822.py` | 8/22 직진 캘리브레이션. **`odom_calibrate` 노드가 대체한다** (파라미터로 직진/회전 둘 다) |
| `ddsm_raw_monitor.py` | 시리얼 원시 모니터. `hw_test` GUI 로 대체 |
| `frames_*.gv`, `frames_*.pdf` | `ros2 run tf2_tools view_frames` 출력물. 그때그때 다시 뽑으면 된다 |
| `my_second_map_r.xcf` | GIMP 편집 파일 |
| `ROS2_SI.png` | 어느 문서에서도 참조하지 않는다 |

---

## 옮기지 않은 것과 그 이유

- **`src/sllidar_ros2/`** — 서드파티 패키지를 통째로 가져온 것이다. 다른 모델용 launch 가
  20개쯤 있지만 **업스트림과 어긋나면 나중에 갱신이 괴로워진다.**
  우리가 쓰는 것은 `sllidar_s2_launch.py` 하나다 (S2 계열, 1,000,000 보드).
- **루트의 `.md` 들** (`GEMINI.md`, `FABLE_REASONING.md`, `read_prompt.md`,
  `first_you_need_it.md`) — 로봇 코드가 아니라 **작업 방식/에이전트 지시문**이다.
- **`src/relayrobot_description/` 패키지** — 로봇 본체. 전부 활성이다.

> `relayrobot_driver` 는 **옮겼다.** 위의 전용 절 참고.
