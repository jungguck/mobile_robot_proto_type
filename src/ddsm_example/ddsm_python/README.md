# ⚠️ ddsm_python — 레거시 실험 폴더 (정본 아님)

**이 폴더의 파이썬 파일은 초기 실험 흔적이다. 여기 있는 값을 그대로 믿지 마라.**
2026-09-16 에 실제로 여기 있는 파일들 때문에 진단이 한참 꼬여서 이 문서를 남긴다.

---

## 정본은 어디인가

| 용도 | **정본 (이걸 써라)** |
|---|---|
| 모터 드라이버 | `src/relayrobot_description/relayrobot_description/motor_drive_1.py` |
| ROS2 드라이버 노드 | `src/relayrobot_description/relayrobot_description/real_robot_driver_260519.py` |
| 직진 캘리브레이션 | 저장소 루트의 `motor_test0822.py` |
| 장치 연결 확인 | 저장소 루트의 `check_devices.sh` |
| **프로토콜 진짜 정본** | `src/ddsm_example/ddsm_example/json_cmd.h` + `ddsm_example.ino` (펌웨어 소스) |

> 프로토콜이 의심되면 **문서보다 펌웨어 소스**를 봐라.
> 명령 사용법 요약은 `docs/DEBUG_LOG_2026-09-16.md` 1절에 정리돼 있다.

---

## 확정된 기준값 (레거시 파일과 대조용)

| 항목 | **맞는 값** |
|---|---|
| `id=1` | **오른쪽 (R)**, 전진 시 **음수** |
| `id=2` | **왼쪽 (L)**, 전진 시 **양수** |
| `wheel_radius` | **0.0325 m** |
| `wheel_base` | **0.22 m** |
| cmd 환산 | `m/s → cmd` 는 **×600** (cmd 단위 = 0.1RPM) |
| 포트 | **`/dev/motor`** (udev 별칭 → `/dev/ttyACM0`) |
| 보드 타입 | `{"T":11002,"type":210}` — ⚠️ 효과 미확인, `DEBUG_LOG_2026-09-16.md` 7절 |

---

## 파일별 주의사항

| 파일 | 상태 | 문제 |
|---|---|---|
| `motor_drive.py` | 레거시 | 초기화가 `{"T":11002,"id":N}` — `T:11002` 는 `id` 가 아니라 **`type`** 을 받는다. 이 형태는 **조용히 무시**된다. cmd 환산도 `×60`(정본 ×600). `wheel_radius=0.035`, `wheel_base=0.2` 로 실측과 다름. 부호 규약도 정본과 반대 |
| `motor_drive_1.py` | **구버전 사본** | 정본과 **파일명이 같아서 가장 위험하다.** 정본은 `relayrobot_description/` 쪽이다. 이쪽은 `×60`, 옛 기구학 상수, 옛 부호 규약 |
| `motor_node.py` | 레거시 | `motor_drive.py`(위 문제들) 를 import |
| `motor_node_1.py` | 레거시 | 포트가 `/dev/ttyACM0` 하드코딩. odom 공분산·상수가 옛 값 |
| **`no_terminal.py`** | **값이 틀림** | **`MOTOR_ID_L=1, MOTOR_ID_R=2` — 좌우가 뒤바뀌어 있다.** `WHEEL_RADIUS=0.07`(정본 0.0325), **`WHEEL_BASE=0.02`(정본 0.22 — 1/10)**. 참고하지 말 것 |
| **`check_feedback.py`** | **포트 틀림** | `/dev/ttyUSB0` 하드코딩 = **IMU 포트다. 모터가 아니다.** 초기화도 `{"T":11002,"id":1}` 형태 |
| **`test_encoder.py`** | **포트 틀림** | `/dev/ttyUSB0` 하드코딩. 다만 주석의 **`total_ticks = laps*32768 + raw_pos`** 는 정확하고 유용하다 (`pos` 되감김 처리) |
| **`enable_test.py`** | **성립 안 함** | 38400 보드레이트로 **raw DDSM 패킷**을 `/dev/ttyACM0` 에 직접 쓴다. 그 포트는 ESP32 의 USB CDC 라 **JSON 을 받는다.** 문서에 없는 `mode 0x08` 사용. 막다른 실험 |
| **`propose_mpc.py`** | **실행 불가** | 존재하지 않는 `motor_driver` 모듈을 import (파일명은 `motor_drive*.py`) |
| `simply.py` | ✅ 쓸만함 | 대화형 JSON 전송 REPL. `python3 simply.py /dev/motor`. 단 **대화형이라 원격 자동화엔 부적합** |
| `serial_simple_ctrl.py` | ✅ 쓸만함 | 위와 비슷. `setRTS(False)`/`setDTR(False)` 를 한다 |
| `requirement.txt` | ✅ | `pyserial==3.5` |

---

## 여기서 뭔가 돌려야 한다면

`simply.py` 만 쓰고, 포트를 **명시**해라.

```bash
python3 simply.py /dev/motor
# 예: {"T":10010,"id":1,"cmd":-294,"act":10}     <- 오른쪽 전진
#     {"T":10010,"id":1,"cmd":0,"act":10}        <- ★ 반드시 정지
```

> ⚠️ 보드 heartbeat 가 기본 비활성(`-1`)이다. **정지 명령을 안 보내면 모터가 영원히 돈다.**
> 프로그램이 죽어도 안 멈춘다.

> ⚠️ 여러 모터에 연속으로 명령을 보낼 때는 **사이에 10ms 간격**을 둬라.
> 반이중 RS485 라 충돌하면 `{"T":20010,"crc":0}` 이 돌아온다.
