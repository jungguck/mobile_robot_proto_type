# 🚦 RUNBOOK — 여기서 시작한다

**세션을 열면 이 문서부터 읽는다.** 뭘 해야 하는지, 뭘 쓰면 되는지, 어디까지 됐는지가 여기 있다.
긴 설명은 `README.md`(설치·STAGE 가이드)와 `docs/DEBUG_LOG_*.md`(그날의 근거)에 있다.

**최종 갱신: 2026-11-10**

---

## 🎯 최종 목표 — 이걸 잊지 말 것

> ### **Cartographer SLAM 으로 지도를 만들고 → A\* 로 경로를 짜고 → Tube-MPC 로 목표까지 자율주행한다.**

지금 하는 모든 일은 **이 한 줄로 가는 계단**이다. 모터 디버깅도, 문서 정리도, 저장소 청소도
그 자체가 목적이 아니다. **어떤 작업을 할지 고민되면 "이게 SLAM 에 가까워지는가" 로 판단한다.**

**SLAM 까지 남은 길 — 하드웨어는 모터 + 라이다 두 개로 확정했다.**

```
✅ STAGE 1  모터        ← 2026-09-16 통과
❌ STAGE 2  IMU         ← 삭제. IMU 를 쓰지 않는다 (2026-11-01 결정)
▶  STAGE 3  LiDAR       ← 여기부터. 장치 인식까지는 됐다
▶  STAGE 4  바퀴 odom   ← 1m 직진 + 90° 회전 검증
⏸  STAGE 5  SLAM        ← 여기가 목표
⏸  STAGE 6  자율주행
```

> ## 2026-11-01 — IMU 를 파이프라인에서 제거했다
>
> **구성은 "바퀴 엔코더 오도메트리 + 라이다" 두 개다.** IMU(EBIMU9DOFV5)는 안 쓴다.
> 관련 코드·설정·udev 규칙을 전부 걷어냈다. 되살릴 일이 생기면 git 이력에서 꺼낸다.
>
> **왜 IMU 없이 SLAM 이 되는가:** Cartographer 설정이 원래부터
> `TRAJECTORY_BUILDER_2D.use_imu_data = false` 였다 — 2D SLAM 은 IMU 를 안 쓴다.
> 발목을 잡던 건 **EKF** 였다. 옛 `ekf.yaml` 은 yaw 소스를 IMU 하나로 통일한 설정이라
> IMU 를 빼면 yaw 관측이 0 개가 되어 로봇이 회전을 못 느꼈다.
> → `ekf.yaml` 을 **바퀴 odom 단독**으로 다시 써서 막았다 (x·y·yaw·v_yaw 전부 사용).
>
> **EKF 를 아예 빼지는 않았다.** `odom→base_link` TF 를 내는 노드가 그것뿐이다.
> 드라이버는 TF 를 일부러 쏘지 않는다(이중 발행 방지). 빼면 TF 사슬이 끊겨 SLAM 이 안 돈다.
>
> **대가:** 바퀴만 쓰면 슬립으로 yaw 가 드리프트한다. Cartographer scan 매칭이 교정해 주는
> 종류의 오차지만, **바퀴가 헛도는 바닥(카펫·문턱·급회전)에서는 지도 품질이 떨어진다.**
> 그리고 `odom_check.py` 의 회전 스케일 자동 계산을 잃었다 — 독립 기준이 없어졌기 때문이다.
> wheel_base 는 이제 **제자리 90° 수동 회전**으로 맞춰야 한다.

> ⚠️ **09-16 에 찾은 저속 헌팅은 SLAM 직결 문제다.** 0.05 m/s 로 달리면 속도가 28% 씩 널뛰어
> 로봇이 뚝뚝 끊긴다 → **odom prior 가 순간적으로 튀고** scan 매칭이 엉뚱한 위치로 수렴한다.
> IMU 를 뺀 지금은 prior 가 바퀴 하나에서만 나오므로 **더 중요해졌다.**
> **순항 속도 0.15 m/s 이상**을 SLAM 주행에 반드시 적용할 것. (`DEBUG_LOG_2026-09-16.md` 6절)

---

## 1. 지금 어디까지 됐나

| 단계 | 상태 | 근거 |
|---|---|---|
| 원격 접속 (`ssh robot`) | ✅ | `docs/REMOTE_ACCESS.md` |
| PC↔젯슨 DDS (Jazzy↔Humble) | ✅ | `DEBUG_LOG_2026-09-07.md` |
| 기구학 상수 확정 | ✅ **재측정 금지** | `DEBUG_LOG_2026-09-03.md` |
| **STAGE 1 — 모터 단독** | ✅ **2026-09-16 통과** | `DEBUG_LOG_2026-09-16.md` |
| STAGE 2 — IMU 단독 | ❌ **삭제** (IMU 미사용 확정) | 2026-11-01 |
| STAGE 3 — LiDAR 단독 | ▶ 장치 인식됨, `/scan` 확인 남음 | |
| STAGE 4 — 바퀴 odom 검증 | ▶ `ekf.yaml` 바퀴 단독으로 재작성 완료 | 2026-11-01 |
| TF 사슬 (map→odom→base_link→lidar) | ✅ 발행자 3개 확인 (코드) | `DEBUG_LOG_2026-11-10.md` |
| 라이다 장착 방향 (`rpy`) | ⚠️ **미검증** — 틀리면 지도가 회전 | `DEBUG_LOG_2026-11-10.md` 4절 |
| STAGE 5 — SLAM | ⏸ | |
| STAGE 5.5 — 리허설(로봇 없이) | ✅ 통과 | `DEBUG_LOG_2026-09-13.md` |
| STAGE 6 — 자율주행 | ⏸ | |

**현재 하드웨어 구성:** 모터 + 라이다. **IMU 는 쓰지 않는다.** 전원 12V, 모터·젯슨 분리 공급.

---

## 2. ★ 다음에 할 일

### 🎯 SLAM 으로 가는 길 (최우선)

- [ ] **젯슨에서 재빌드** — `ekf.yaml`·launch 가 바뀌었다. 이걸 안 하면 옛 설정이 돈다
      `ssh robot 'cd ~/mobile_robot_proto_type && git pull && colcon build --packages-select relayrobot_description'`
- [ ] **udev 규칙 재적용** — IMU 규칙을 뺐다
      `ssh robot 'cd ~/mobile_robot_proto_type && sudo cp src/relayrobot_description/scripts/99-robot-devices.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger'`
- [ ] **`./check_devices.sh` 로 라이다·모터 2개 ✅ 확인**
- [ ] **STAGE 3 — LiDAR 단독** (`/scan` 10Hz)
      RViz 로 볼 때는 **Fixed Frame 을 `lidar_v1_1`** 로 — TF 없이도 보인다
- [ ] **⚠️ 라이다 장착 방향 확인** (미검증, SLAM 직결)
      URDF `Rigid 40` 이 `rpy="0 0 0"` = "라이다 0° 가 로봇 전진(+x)과 일치" 를 전제한다.
      RViz Fixed Frame=`base_link` → 로봇 **앞**에 물체를 두고 점이 **+x 에 찍히는지** 본다.
      뒤에 찍히면 `rpy="0 0 3.14159"` 로 고친다. 틀리면 **지도가 통째로 회전한다.**
      (근거: `DEBUG_LOG_2026-11-10.md` 4절)
- [ ] **STAGE 1 재확인 — 모터** (USB 를 다시 꽂았으므로) `python3 tools/motor_diag.py`
- [ ] **STAGE 4 — 바퀴 odom 검증**
      직진: `python3 tools/odom_check.py` → 1m 직진 → ① 선속도 스케일 0.9~1.1
      회전: reset → 제자리 90° 회전 → `ODOM_RAW` 의 θ 가 90° 인지 → 어긋나면 `wheel_base` 보정
- [ ] **STAGE 5 — SLAM** ← 목표. `cartographer.launch.py` 로 바로 간다 (새로 쓸 코드 없음)
- [ ] **순항 속도 0.15~0.2 m/s 를 MPC·경로계획 설정에 반영** — 저속이면 SLAM 이 깨진다

### 모터 마무리 (SLAM 전에 끝내두면 좋음)

- [ ] **줄자로 실제 주행거리 측정 → `wheel_radius` 보정**
      `python3 tools/drive_straight.py --dist 0.50 --speed 0.20 --measured <실측m>`
      → odom 정확도는 EKF·SLAM 품질에 직접 영향
- [ ] **왼쪽 모터 속도 산포 재측정** — 09-16 표가 오른쪽 기준이다
      `python3 tools/motor_diag.py --sweep`
- [ ] `tools/motor_diag.py`·`tools/drive_straight.py` 실기 검증 (문법 검사만 된 상태)

### 곁가지 — SLAM 을 막고 있지 않다. 시간 남을 때만

- [ ] **젯슨 저장소 최신화** — `42f5166` → PC 는 `0f416b7` (3커밋 뒤)
      `ssh robot 'cd ~/mobile_robot_proto_type && git pull'`
- [ ] **저장소 정리** → **[`docs/CLEANUP.md`](CLEANUP.md)**
      (22MB 벤더 zip, 참조 0 인 이미지/TF덤프, 틀린 레거시 스크립트, 중복 README)
      ⚠️ `src/ddsm_example/mpc_tubempc/` 는 **절대 지우지 말 것** — Tube-MPC 핵심 모듈이다

---

## 3. 시작 절차

### 전원 넣는 순서
```
USB 전부 꽂기  →  모터 12V  →  젯슨 전원 (마지막)
```

### 붙었는지 확인
```bash
# [PC] 순서대로. 끊긴 층이 곧 원인이다 (진단 사다리)
ping -c 2 -W 2 172.30.1.45                      # ① 같은 망에 있나
ssh -o BatchMode=yes robot 'echo OK'            # ② SSH 키 인증
ssh robot 'echo $ROS_DISTRO $ROS_DOMAIN_ID'     # ③ humble / 0
ssh robot 'cd ~/mobile_robot_proto_type && ./check_devices.sh'   # ④ USB 장치
```

> ⚠️ **부팅 직후 1분쯤은 USB 열거가 안 끝났다.** 장치가 없다고 바로 판단하지 말고 다시 확인할 것.

### 기동

```bash
# [젯슨] 전체 (모터 + 라이다 + 바퀴 odom + EKF)
ros2 launch relayrobot_description real_robot_260519.launch.py

# 오도메트리만 볼 때 (라이다 없이)
ros2 launch relayrobot_description real_robot_260519.launch.py use_lidar:=false

# [젯슨-2] SLAM 은 따로 띄운다
ros2 launch relayrobot_description cartographer.launch.py
```

> IMU 인자(`use_imu`)는 **없다.** IMU 를 쓰지 않는 구성이다.

---

## 4. ★ 어떤 스크립트를 쓸 것인가

**증상에 따라 고른다.** 헷갈리면 이 표만 보면 된다.

| 상황 | 쓸 것 | 왜 |
|---|---|---|
| **모터가 아예 안 도는 것 같다** | `python3 tools/motor_diag.py` | 엔코더 절대위치로 **도는지 안 도는지 확정**. 좌우 균형까지 |
| **떨린다 / 뚝뚝 끊긴다** | `python3 tools/motor_diag.py --sweep` | 속도별 산포. 저속 헌팅 구간 확인 |
| **직진/후진 시켜보고 싶다** | `python3 tools/drive_straight.py --dist 0.3 --speed 0.15` | 전후진 둘 다 됨 + 떨림 통계 |
| **`wheel_radius` 캘리브레이션** | `tools/drive_straight.py --measured <m>` 또는 `motor_test0822.py --measured <m>` | 보정계수 계산 |
| **보드가 응답을 하긴 하나?** | `python3 ddsm_raw_monitor.py` | raw 응답을 그대로 본다. Ctrl+C 로 종료 |
| **명령을 손으로 하나씩 쳐보고 싶다** | `python3 src/ddsm_example/ddsm_python/simply.py /dev/motor` | 대화형 REPL |
| **ROS 통합 상태로 돌리고 싶다** | `ros2 run relayrobot_description real_robot_driver_260519` | STAGE 1 정식 경로 |

### `motor_test0822.py` vs `ddsm_raw_monitor.py` — 뭐가 맞나

**둘 다 용도가 다르다.**

- **`ddsm_raw_monitor.py`** = **보드가 살아있나** 를 보는 것. 모터가 안 돌 때 **먼저** 쓴다.
  raw 응답을 그대로 찍는다. 무한 루프라 **Ctrl+C 로 직접 끝내야 한다** → 원격 자동화엔 부적합.
- **`motor_test0822.py`** = 모터가 도는 게 **확인된 뒤** 쓰는 **직진 캘리브레이션**.
  ⚠️ **전진만 된다.** 음수 거리를 못 받는다 (후진은 `tools/drive_straight.py`).

> **권장:** 진단은 `tools/motor_diag.py` 로 시작해라. 위 둘보다 판정이 명확하다
> (추측이 아니라 엔코더 누적치로 "돌았다/안 돌았다" 를 찍어준다).

---

## 5. 모터에 명령 보내기 — 최소한만

경로: `JSON 한 줄 → /dev/motor(115200) → ESP32 → RS485 → 모터`. **모터에 직접 보내는 게 아니다.**

| | |
|---|---|
| 구동 | `{"T":10010,"id":N,"cmd":V,"act":10}` — `cmd` 단위 **0.1RPM**, 범위 ±2100 |
| 정지 | `{"T":10010,"id":N,"cmd":0,"act":10}` |
| 모드 | `{"T":10012,"id":N,"mode":2}` — 0=개루프, 2=속도루프, 3=위치 |
| 상태 | `{"T":10032,"id":N}` → `mil`(회전수), `pos`(0~32767) |

| ID | 위치 | 전진 부호 |
|---|---|---|
| `id=1` | **오른쪽 (R)** | **음수** |
| `id=2` | **왼쪽 (L)** | **양수** |

세 가지만 기억하면 사고가 안 난다:

1. **정지 명령을 꼭 보낸다.** heartbeat 가 비활성이라 프로그램이 죽어도 모터는 계속 돈다.
2. **모터 사이에 10ms 간격.** 반이중 RS485 라 충돌하면 `{"T":20010,"crc":0}`.
3. **`pos` 는 1회전마다 되감긴다.** 누적은 `mil*32768 + pos`.

전체 명령 일람: `docs/DEBUG_LOG_2026-09-16.md` 1절.

---

## 6. 끄는 절차

```bash
ssh robot robot-off          # 모터정지 → 노드종료 → sync → poweroff
ping -c 25 -W 1 172.30.1.45  # 무응답 확인
# LED·팬 정지 확인 후 → 12V 모터 전원 차단
```

- **젯슨 먼저, 모터 나중.** 모터가 도는 중에 젯슨을 끄면 정지시킬 수단이 사라진다.
- 젯슨은 **SD 카드 부팅**이다. 라이브로 전원을 뽑으면 카드가 깨진다. 실제로 한 번 깨졌다.

---

## 7. 막혔을 때

**아래에서 위로 올라간다. 끊긴 층이 곧 원인이다.**

```
① 핑 → ② SSH → ③ 젯슨 ROS → ④ 노드 → ⑤ USB 장치 → ⑥ 모터 구동
```

모터가 안 돌면 이 순서로:

1. `tools/motor_diag.py` — **정말 안 도는지** 확인 (`pos` 되감김 오판 배제)
2. 개루프(`mode 0`)에서도 전류 `crt` 에 DC 성분이 없는지 → 없으면 구동 전원 의심
3. 여러 모터에 명령 보낼 때 **10ms 간격**을 뒀는지
4. 전원을 껐다 켜고 재시도

> ❌ **하지 말 것:** "`type:210` 을 안 보내서 그렇다" 고 의심하기.
> 2026-09-16 에 재현되지 않았다. `type:115` 로도 정상 회전했다.
> ❌ **하지 말 것:** `act` 나 명령 재전송 주기로 떨림 잡기. 전수 스윕으로 배제됐다.
> ❌ **하지 말 것:** 기구학 상수(`wheel_radius`/`wheel_base`/`×600`) 재의심. 09-03 확정값이다.

### 알려진 함정

| 함정 | 내용 |
|---|---|
| 젯슨 Python | **3.10** 이다. f-string 따옴표 중첩은 3.12+ 문법 — PC 에서 통과해도 젯슨에서 죽는다 |
| 원격 스크립트 | 보내기 전 젯슨에서 문법 검사: `ssh robot 'python3 -c "import ast; ast.parse(open(\"/tmp/x.py\").read())"'` |
| `ddsm_python/` 폴더 | **전부 레거시다.** `no_terminal.py` 는 좌우 ID 가 뒤바뀌어 있다. `README.md` 참고 |
| PC conda | `conda deactivate` 를 안 하면 ROS 파이썬을 가린다 |
| PC IP | DHCP 라 바뀐다. 젯슨은 `.45` 유지 중 |

---

## 8. ★ 세션을 끝낼 때 (이걸 해야 다음에 안 헤맨다)

1. **`docs/DEBUG_LOG_<날짜>.md`** 를 쓴다 — 한 줄 요약 / 한 것 / **미해결** / 밟은 함정
2. **이 RUNBOOK 의 1절(상태)과 2절(다음 할 일)을 갱신한다** ← 가장 중요
3. 틀린 것을 찾았으면 **원래 문서를 고친다.** 주석으로 덧붙이지 말고 본문을 고칠 것
4. 임시 스크립트가 쓸만하면 **`tools/` 에 남긴다.** `/tmp` 에 두면 재부팅 때 사라진다
5. `docs/wiki/log.md` 에 한 항목 추가
