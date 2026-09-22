# 🧭 TF Coordinate System

로봇이 "내가 어디 있는가" 를 말하는 방식. 이 프로젝트에서 **가장 자주 사고가 나는 지점**이다.

## 트리 구조

```
map ──[cartographer_node]──► odom ──[드라이버 또는 ekf_filter_node]──► base_link
                                      └─[robot_state_publisher]──► lidar_v1_1
```

| 변환 | 발행자 | 의미 | 성질 |
|------|--------|------|------|
| `map → odom` | `cartographer_node` | SLAM 이 계산한 **누적 드리프트 보정분** | 불연속(점프). loop closure 때 확 바뀜 |
| `odom → base_link` | **구성에 따라 다름** (아래) | 추측항법 | 연속·매끄러움. 단 드리프트 누적 |
| `base_link → lidar_v1_1` | `robot_state_publisher` | URDF 고정 장착 위치 | 상수 |

**`odom → base_link` 를 누가 내는가는 `use_imu` 가 정한다 (2026-09-22):**

| 구성 | 발행자 | 내용 |
|---|---|---|
| **기본 (`use_imu:=false`)** | `real_robot_driver_260519` | 바퀴 인코더 적분만 |
| `use_imu:=true` | `ekf_filter_node` | 바퀴 + IMU 융합 |

IMU 가 없어도 되는 이유: 2D SLAM 에서 yaw 를 실제로 잡는 주체는 IMU 가 아니라
**라이다 scan matching** 이다. 바퀴 yaw 는 스캔 사이를 메우는 초기 추정값일 뿐이고,
드리프트는 `map → odom` 이 흡수한다.

**두 프레임의 역할이 다르다:**
- `odom` — **매끄럽지만 틀린다.** 점프가 없어 제어 미분에 안전. 시간이 지나면 진짜 위치에서 멀어짐.
- `map` — **점프하지만 맞는다.** 전역 좌표. 지도·목표·경로는 전부 여기 기준.

## ★ 프레임을 섞으면 생기는 일 (2026-09-13 수정 전 실제 구조)

`bridge_node` 는 `/odom` 토픽의 pose(= **odom 프레임**)를 현재 위치로 썼는데,
`path_planner` 가 발행하는 `/global_path` 와 사용자가 주는 `/mpc_goal` 은 **map 프레임**이었다.
두 노드 어디에도 tf2 가 없어서 **변환 없이 그대로 뺐다.**

```
추종오차 = (map 프레임 경로점) - (odom 프레임 현재위치)
         = 진짜오차 + (map→odom)          ← 드리프트 보정분이 통째로 섞여 들어옴
```

SLAM 이 드리프트를 보정할수록 `map→odom` 이 커지므로 **주행할수록 오차가 커진다.**
시작 직후에는 `map ≈ odom` 이라 "잘 되는 것처럼" 보이는 게 특히 나쁘다.

**해결:** `bridge_node` 와 `path_planner` 모두 토픽 대신
**`global_frame`(기본 `map`) → `base_frame`(`base_link`) TF 조회**로 현재 위치를 받는다.
목표·경로·위치가 전부 같은 프레임이 되어 뺄셈이 그대로 유효해진다.

```python
tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
```

`-p global_frame:=odom` 으로 내리면 SLAM 없이도 같은 코드가 돈다(전역 보정만 없을 뿐).

## ⚠️ TF 조회에 timeout 을 주면 안 된다

```python
lookup_transform(..., timeout=Duration(seconds=0.1))   # ❌
lookup_transform(...)                                   # ✅
```

`rclpy.spin()` 은 **단일 스레드**다. timeout 이 있으면 그 함수가 내부에서 자는데,
자는 동안 `/tf` 메시지가 **배달되지 않는다.** 즉 **기다려도 절대 성공할 수 없고**
제어 주기(10Hz = 100ms)만 통째로 잡아먹는다. TF 가 없으면 즉시 실패시키고
다음 주기에 다시 시도하는 쪽이 맞다. (`MultiThreadedExecutor` 로 가면 얘기가 달라진다.)

## ⚠️ PC 와 젯슨의 시계가 맞아야 한다

TF 는 타임스탬프로 보간한다. 원격 운영에서 두 기계의 시계가 어긋나면
PC RViz 에서 `TF extrapolation` 에러가 나고 **지도가 아예 안 그려진다.**

```bash
ssh robot 'date -u +%s.%N'; date -u +%s.%N     # 차이 1초 이내
```

어긋나 있으면 젯슨에 `chrony` 설치. (README `STAGE 0`)

## ⚠️ `odom → base_link` 를 두 곳에서 발행하지 말 것

두 발행자가 경쟁하면 RViz 에서 로봇이 떨리고, 원인 추적이 매우 어렵다.
자세한 사고 기록은 [[Debugging_Experience]] 1번.

**2026-09-22 부터 이 책임은 `publish_tf` 파라미터가 정한다.** launch 의 `use_imu` 가
드라이버와 `ekf_node` 중 정확히 하나만 켜지도록 짝지어 놓았다.

| | 드라이버 `publish_tf` | `ekf_node` |
|---|---|---|
| `use_imu:=false` (기본) | **True** | 안 띄움 |
| `use_imu:=true` | False | 띄움 (`publish_tf: true`) |

실수하기 쉬운 두 경로:
- `ros2 run relayrobot_driver main_driver` (= `motor_node_1.py`) — `odom` 과 TF 를
  **직접** 쏜다. 띄우지 말 것. 기구학 상수도 낡았다 (`r=0.05`, `base=0.165`).
- `hw_test` GUI 의 **EKF Start 버튼** — 기본 구성에서 누르면 드라이버와 겹친다.
  그래서 `use_ekf=False` 일 때는 막아뒀다.

확인:
```bash
ros2 topic info /tf --verbose     # odom→base_link 발행 노드가 하나인지
ros2 run tf2_tools view_frames
```

## 🔗 관련 문서
- [[System_Architecture]]
- [[MPC_Controller]]
- [[Debugging_Experience]]
- 상세: `docs/DEBUG_LOG_2026-09-13.md` 12-3 절
