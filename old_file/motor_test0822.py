#!/usr/bin/env python3
"""
모터 구동 + 바퀴 오도메트리 직진 캘리브레이션 테스트 (2026-08-22)
================================================================

목적
----
지정한 거리만큼 직진시킨 뒤, 바퀴 엔코더가 적분한 거리(odom)와
줄자로 잰 실제 거리를 비교해서 wheel_radius 가 맞는지 확인한다.

  실제거리 / odom거리 = 보정계수
  wheel_radius_new = wheel_radius * 보정계수

사용법
------
  python3 motor_test0822.py                      # 기본 50cm, 0.1 m/s
  python3 motor_test0822.py --dist 1.0           # 1m 주행
  python3 motor_test0822.py --dist 0.5 --speed 0.15
  python3 motor_test0822.py --measured 0.60      # 실측 60cm 입력 -> 보정계수 계산

  ※ 라이다/EKF 필요 없음. 드라이버 노드도 띄우지 말 것 (/dev/motor 를 점유함).


2026-08-22 이 날 확인된 하드웨어 사실들
--------------------------------------
1) 보드 초기화는 {"T":11002,"type":210} 이어야 한다.
   펌웨어(ddsm_example.ino: set_ddsm_type)는 115/210 만 인정하고,
   DDSM400 은 DDSM210 계열 프로토콜을 쓴다. 전원 사이클당 1회 필요.
   기존 코드의 {"T":11002,"id":N} 은 타입 설정이 아니라서, 이걸 빼먹으면
   보드가 구동 명령(T:10010)에 응답조차 하지 않는다.

2) cmd / spd 의 단위는 RPM 이 아니라 0.1RPM 이다.
   실측: cmd 100 -> spd 105 회신 (= 약 10 RPM).
   따라서 m/s -> cmd 변환은 x60 이 아니라 x600.

3) 모터 ID 와 물리 위치: id=1 -> 오른쪽(R), id=2 -> 왼쪽(L).

4) 전진 부호 규약: R 은 음수, L 은 양수.
   좌우 허브모터가 거울 대칭으로 장착돼 있어서 그렇다.
   MotorDriver.DIR_R/DIR_L 이 이 부호를 흡수하므로,
   drive() 와 read_feedback() 모두 "+ = 로봇 전진" 으로 통일돼 있다.

5) 보드 heartbeat 는 기본 비활성(-1)이다.
   정지 명령을 보내지 않으면 모터가 무한히 돈다. 노드가 죽어도 안 멈춘다.


[중요] 정지 구간을 왜 이렇게 짰는가
-----------------------------------
MotorDriver.read_feedback() 은 새 피드백 프레임이 안 오면
직전 rpm 값을 그대로 돌려준다. 그런데 이 보드는 "명령을 받았을 때만"
피드백을 보낸다. 따라서 정지 명령을 한 번만 보내고 가만히 있으면,
로봇은 이미 멈췄는데도 옛날 속도값이 계속 적분되어 거리가 부풀려진다.

실제로 이 버그 때문에 첫 측정(50cm 목표)에서
  - 정지 명령 시점 odom : 50.4 cm
  - 1초 더 적분한 값    : 59.8 cm
로 9.4cm 가 유령처럼 늘어났다.

그래서 아래 settle 구간에서는 정지 명령을 계속 재전송해서
피드백이 계속 들어오게 만든다. 그러면 감속/정지가 피드백에
그대로 찍히고, 관성 주행분만 정확히 잡힌다.
"""

import argparse
import math
import os
import sys
import time

# 이 스크립트는 저장소 루트에서 실행하는 것을 전제로 한다.
# MotorDriver 는 relayrobot_description 패키지 안에 있으므로 경로를 잡아준다.
_PKG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'src', 'relayrobot_description')
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

from relayrobot_description.motor_drive_1 import MotorDriver  # noqa: E402


# spd 피드백 -> 바퀴 선속도(m/s)
#   spd 단위가 0.1RPM 이므로 /10 -> RPM, /60 -> RPS, 합쳐서 /600
def spd_to_ms(spd, wheel_radius):
    return (spd / 600.0) * (2 * math.pi * wheel_radius)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/motor')
    ap.add_argument('--dist', type=float, default=0.50, help='목표 거리 (m)')
    ap.add_argument('--speed', type=float, default=0.10, help='주행 속도 (m/s)')
    ap.add_argument('--radius', type=float, default=0.0325, help='바퀴 반지름 (m)')
    ap.add_argument('--base', type=float, default=0.22, help='양 바퀴 중심거리 (m)')
    ap.add_argument('--countdown', type=int, default=8, help='출발 전 대기 (초)')
    ap.add_argument('--settle', type=float, default=2.0, help='정지 후 관측 (초)')
    ap.add_argument('--measured', type=float, default=0.0,
                    help='줄자로 잰 실제 거리 (m). 주면 보정계수를 계산해준다')
    a = ap.parse_args()

    d = MotorDriver(port=a.port, wheel_radius=a.radius, wheel_base=a.base)
    if d.ser is None:
        print("시리얼 연결 실패. /dev/motor 확인, 드라이버 노드가 점유 중인지 확인.")
        return
    time.sleep(0.3)
    d.read_feedback()      # 초기화 응답 찌꺼기 비우기

    print(f"\n목표 {a.dist*100:.0f} cm, 속도 {a.speed} m/s, r={a.radius} m")
    for i in range(a.countdown, 0, -1):
        print(f"  시작 {i}초 전... (바퀴 접지점을 바닥에 표시)", flush=True)
        time.sleep(1.0)

    print("\n>>> 출발\n", flush=True)

    dist = 0.0
    t_prev = t0 = time.time()

    # --- 주행 구간: 목표 거리까지 ---
    while dist < a.dist:
        d.drive(a.speed, 0.0)
        time.sleep(0.05)
        l, r = d.read_feedback()
        now = time.time()
        dt, t_prev = now - t_prev, now
        v = (spd_to_ms(l, a.radius) + spd_to_ms(r, a.radius)) / 2.0
        dist += v * dt
        if now - t0 > 30:
            print("  [중단] 30초 초과 — 바퀴가 안 도는지 확인")
            break

    dist_at_stop = dist
    t_stop = time.time()
    print(f">>> 목표 도달 -> 정지 명령 "
          f"(주행 {t_stop-t0:.2f}s, odom {dist_at_stop*100:.1f} cm)")

    # --- settle 구간: 정지 명령을 '계속' 재전송해서 피드백을 살려둔다 ---
    #     (재전송하지 않으면 stale 값이 적분되어 거리가 부풀려진다)
    v_last = None
    while time.time() - t_stop < a.settle:
        d.drive(0.0, 0.0)
        time.sleep(0.05)
        l, r = d.read_feedback()
        now = time.time()
        dt, t_prev = now - t_prev, now
        v_last = (spd_to_ms(l, a.radius) + spd_to_ms(r, a.radius)) / 2.0
        dist += v_last * dt

    d.stop()
    d.close()

    coast = dist - dist_at_stop
    print("\n" + "=" * 56)
    print(f" 정지 명령 시점 odom : {dist_at_stop*100:6.1f} cm")
    print(f" 관성 주행분         : {coast*100:6.1f} cm")
    print(f" odom 총 이동거리    : {dist*100:6.1f} cm")
    print(f" 정지 후 잔여 속도   : {(v_last or 0)*100:6.1f} cm/s"
          f"   (0 에 가까워야 정상)")
    print(f" 총 소요 시간        : {time.time()-t0:6.2f} s")
    print("=" * 56)

    if a.measured > 0:
        k = a.measured / dist if dist > 1e-6 else 0.0
        print(f"\n 실측 거리           : {a.measured*100:6.1f} cm")
        print(f" 보정계수            : {k:.4f}")
        print(f" wheel_radius        : {a.radius:.5f} -> {a.radius*k:.5f}")
        print(f"\n 반영 방법:")
        print(f"   ros2 run relayrobot_description real_robot_driver_260519 \\")
        print(f"     --ros-args -p wheel_radius:={a.radius*k:.5f}")
    else:
        print("\n 줄자로 실제 거리를 재서 --measured 0.60 처럼 다시 주면")
        print(" 보정계수를 계산해준다. (모터를 다시 돌리지 않으려면 --dist 0 불가하니,")
        print("  같은 조건으로 한 번 더 주행하면서 계산하게 된다)")


if __name__ == '__main__':
    main()
