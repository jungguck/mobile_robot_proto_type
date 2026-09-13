# 🎯 MPC Controller (Tube-MPC)

경로를 따라가는 제어기. `/global_path` 를 받아 `/cmd_vel` 을 만든다.

- 구현: `src/mpc_tubempc_bridge/src/mpc_tubempc_bridge/bridge_node.py` (ROS 노드)
- 알고리즘: `src/ddsm_example/mpc_tubempc/TubeMPCPlanner.py` (ROS 비의존)

> ⚠️ `mpc_tubempc` 는 ROS 패키지가 아니다. `bridge_node.py` 가 `sys.path` 로 소스 트리를
> 직접 참조하므로 **`colcon build --symlink-install` 이 필수**다. 빼면 `ImportError`.

## 구조

```
        참조 궤적 qRef, 참조 입력 uRef   (A* 경로에서 생성 — 피드포워드)
                    │
   e_act = 현재위치와 qRef[:,0] 의 오차 (로봇 프레임)
                    │
        ┌───────────┴───────────┐
   명목 시스템 e_nom          실측 e_act
        │ QP 풀이                │
      u_nom              ancillary: -K(e_act - e_nom)
        └───────────┬───────────┘
              cmd = uRef + u_corr  (속도 한계로 클램프)
```

- **명목(nominal)**: 외란이 없다고 가정한 이상적 궤적. QP 가 여기를 푼다.
- **tube**: 외란이 있어도 실측이 명목 주위 일정 반경(tube) 안에 머물도록 제약을 미리 타이트닝.
- **ancillary**: 실측과 명목의 편차를 LQR gain `K` 로 되먹여 tube 안에 가둔다.

## ★ QP 가 100% infeasible 이던 이유 (2026-09-13 측정으로 확정)

**증상:** A* 경로는 정상인데 `/mpc/status` 가 계속 `QP_FAILED`. 로봇이 조금 가다 선다.

**원인:** 상태 제약 `x_min` 의 heading 성분이 `-0.3` 이었다. 이 값은 tube 크기만큼
**타이트닝**되므로 실효 한계는 `0.3 - 0.05 = 0.25 rad`(약 14°) 다.
그런데 A* 경로를 따라갈 때 **첫 참조점의 접선 방향과 로봇의 현재 방향은 출발 시 쉽게 20~30° 벌어진다**
(경로가 옆으로 나가면 항상). 즉 **첫 사이클부터 상태 제약 위반 → 항상 infeasible.**

오프라인 재현으로 경계를 측정했다:

```
타이트닝된 상태상한 x_b : [1.95 1.95 0.25 ...]
  e_theta = 0.25 rad -> OK
  e_theta = 0.30 rad -> FAIL      ← 경계가 정확히 0.25
  e_theta = 0.43 rad -> FAIL      ← 실주행에서 관측된 값
```

**해결:** `error_yaw_limit` 파라미터로 분리하고 기본값 `3.2`(사실상 무제한).
**heading 오차는 안전 제약이 아니다** — 돌아서 줄이면 되는 값이다.
실제로 지켜야 할 것은 위치 오차(tube)이고 그건 `error_xy_limit` 과 ancillary 가 담당한다.

## 계산 부하

변수 8개(= `nu 2 × Np 4`), 제약 40행. **작은 QP** 다.

| 기계 | 평균 | 최대 | 10Hz 예산(100ms) 대비 |
|------|------|------|----------------------|
| PC (i5-12400F) | 4.5 ms | 7.3 ms | 5% |
| 젯슨 Orin Nano | *미측정* | — | 파이썬 오버헤드가 대부분이라 몇 배 느려도 여유 |

대부분이 OSQP 자체가 아니라 **cvxpy 의 문제 구성 오버헤드**다.

## 참조 궤적 생성 — 셀 스냅 금지

`_generate_reference_from_path()` 는 경로 위에서 `step_len = max_v × Ts` 씩 전진한 점을 만든다.

예전에는 **셀 인덱스 단위로 건너뛰었다.** A* 한 셀은 0.05m(대각 0.07m)인데 `step_len` 은
0.01~0.02m 라, 한 스텝에 최소 한 셀을 통째로 넘어가 **참조 궤적이 속도 한계의 5~7배로 달아났다.**

- 예측 모델(`_A_matrix(uRef…)`)과 참조 상태가 서로 안 맞아 **도달 불가능한 궤적을 향해 QP 를 푼다**
- `uRef[0]` 이 **항상 `max_v` 에 붙어**, 경로가 90° 꺾여도 전속 전진 명령이 나간다(제자리 회전 대신 튀어나감)

→ `_path_pose_at()` 로 **세그먼트 안에서 선형 보간**하도록 교체.

## ★ 경로가 없으면 선다 (안전)

`use_global_path` 모드에서 `/global_path` 가 없거나 낡으면(`path_timeout`, 기본 2s)
**정지 명령 + `/mpc/status = NO_PATH`** 다.

예전에는 목표까지 **직선 참조**를 만들어 그대로 달렸다. 그런데 A* 실패
("경로 없음", "목표가 막힘")나 계획기 사망도 전부 "경로 없음" 이다. 즉
**갈 수 없다고 판정된 목표를 향해 지도를 무시하고 장애물을 뚫고 직진**하는 구조였다.
경로 추종 모드에서 경로가 없으면 서는 것이 맞다.

## 원격 관측 토픽

제어 루프는 **젯슨에서** 돈다(무선 너머 10Hz 는 곧 제어 지터). PC 는 아래로 관찰한다.

| 토픽 | 타입 | 보는 법 |
|------|------|---------|
| `/mpc/reference_path` | `nav_msgs/Path` | RViz — 지금 호라이즌이 향하는 곳 |
| `/mpc/tracking_error` | `geometry_msgs/Vector3` | `rqt_plot` — e_act (x, y, θ) |
| `/mpc/status` | `std_msgs/String` | `RUNNING` / `QP_FAILED` / `GOAL_REACHED` / `NO_POSE` / `NO_PATH` |

## 주요 파라미터

| 이름 | 기본 | 의미 |
|------|------|------|
| `global_frame` | `map` | 위치·목표·경로 기준 프레임. SLAM 없이 쓰려면 `odom` |
| `velocity_limit` / `omega_limit` | 0.2 / 1.0 | 명령 상한 (첫 실주행은 0.1 권장) |
| `horizon` | 4 | augmented 모델이 4스텝 고정이라 4 권장 |
| `error_yaw_limit` | 3.2 | heading 오차 한계. **낮추면 QP 가 infeasible 해진다** |
| `error_xy_limit` | 2.0 | 위치 오차 한계 (진짜 tube 제약) |
| `path_timeout` | 2.0 | 이보다 낡은 `/global_path` 는 없는 것으로 본다 |
| `goal_tolerance` | 0.1 | 도달 판정 반경 |

> 도달 판정은 **A\* 경로의 마지막 점** 기준이다. 그 점은 격자 중심이라 목표에서
> 최대 반 셀(0.025m) 어긋나 있다 — 도달 오차가 `goal_tolerance` 보다 조금 큰 것은 정상.

## 검증 방법 — 로봇 없이

`tools/mpc_sim.py` 가 가짜 지도 + 차동구동 적분 + TF 를 발행해 **닫힌 루프**를 만든다.
`map→odom` 을 일부러 틀어(`-p map_odom_x:=0.5`) 프레임 정합까지 시험할 수 있다.
절차는 README `STAGE 5.5`.

## 🔗 관련 문서
- [[TF_Coordinate_System]]
- [[System_Architecture]]
- [[Debugging_Experience]]
- 상세: `docs/DEBUG_LOG_2026-09-13.md` 13·14·17 절
