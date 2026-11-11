#!/usr/bin/env python3
"""직진 주행 — 전진/후진 + 떨림(속도 산포) 측정.

젯슨에서, 저장소 루트 기준으로 실행한다.

  python3 tools/drive_straight.py --dist 0.30 --speed 0.15      # 전진 30cm
  python3 tools/drive_straight.py --dist -0.30 --speed 0.15     # 후진 30cm
  python3 tools/drive_straight.py --dist 0.50 --speed 0.20 --measured 0.52
                                                                 # 줄자 실측 주면 보정계수 계산

왜 motor_test0822.py 가 아니라 이것인가
---------------------------------------
`motor_test0822.py` 는 `while dist < a.dist` 구조라 **음수 거리(후진)를 못 받는다.**
후진하면 dist 가 음수로 가서 목표에 영원히 도달하지 못하고 30초 캡에 걸린다.
여기서는 abs(dist) 로 판정해 전후진을 모두 지원하고, 떨림 진단용 통계를 덧붙였다.

캘리브레이션 목적이면 `motor_test0822.py` 를 써도 된다 (전진만).

주의
----
- 드라이버 노드가 /dev/motor 를 점유 중이면 안 된다. 노드를 먼저 끄고 실행할 것.
- 정지 구간에서 정지 명령을 **계속 재전송**한다. 이 보드는 명령을 받을 때만 피드백을
  보내므로, 재전송하지 않으면 옛 속도가 계속 적분되어 거리가 부풀려진다.
- 순항 속도는 0.15 m/s 이상을 권장한다. 저속에서 속도루프가 심하게 헌팅한다.
  (0.05 m/s 변동 28% vs 0.20 m/s 2% — docs/DEBUG_LOG_2026-09-16.md 6절)
"""
import argparse
import math
import os
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PKG = os.path.join(_ROOT, 'src', 'relayrobot_description')
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from relayrobot_description.motor_drive_1 import MotorDriver  # noqa: E402

SETTLE_AFTER = 1.5      # 이 시각 이후 샘플만 "정상상태" 로 본다 (출발 가속 제외)


def spd_to_ms(spd, r):
    return (spd / 600.0) * (2 * math.pi * r)


def stats(samples, after=0.0):
    xs = [v for (t, v) in samples if t >= after]
    if not xs:
        return '샘플 없음'
    n = len(xs)
    mean = sum(xs) / n
    sd = (sum((x - mean) ** 2 for x in xs) / n) ** 0.5
    pct = (100 * sd / abs(mean)) if mean else 0
    return ('n=%3d 평균 %7.1f 표준편차 %5.1f (변동 %3.0f%%) 범위 %d~%d'
            % (n, mean, sd, pct, min(xs), max(xs)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dist', type=float, default=0.30, help='목표 거리(m). 음수면 후진')
    ap.add_argument('--speed', type=float, default=0.15, help='속력(m/s). 0.15 이상 권장')
    ap.add_argument('--radius', type=float, default=0.0325)
    ap.add_argument('--base', type=float, default=0.22)
    ap.add_argument('--countdown', type=int, default=3)
    ap.add_argument('--settle', type=float, default=2.0)
    ap.add_argument('--measured', type=float, default=0.0,
                    help='줄자로 잰 실제 거리(m). 주면 wheel_radius 보정계수를 계산한다')
    a = ap.parse_args()

    sign = 1.0 if a.dist >= 0 else -1.0
    target = abs(a.dist)
    v_cmd = sign * abs(a.speed)

    if abs(a.speed) < 0.15:
        print('  ⚠️ %.2f m/s 는 저속 헌팅 구간이다. 떨릴 수 있다 (0.15 m/s 이상 권장).'
              % abs(a.speed))

    d = MotorDriver(port='/dev/motor', wheel_radius=a.radius, wheel_base=a.base)
    if d.ser is None:
        print('시리얼 연결 실패. /dev/motor 확인, 드라이버 노드가 점유 중인지 확인.')
        return 1
    time.sleep(0.3)
    d.read_feedback()

    print('')
    print('목표 %.0f cm %s, 속도 %.3f m/s, r=%s'
          % (target * 100, '전진' if sign > 0 else '후진', abs(a.speed), a.radius))
    for i in range(a.countdown, 0, -1):
        print('  시작 %d초 전... (바퀴 접지점을 바닥에 표시)' % i, flush=True)
        time.sleep(1.0)
    print('')
    print('>>> 출발')
    print('')

    dist = 0.0
    t_prev = t0 = time.time()
    sl, sr = [], []

    try:
        while abs(dist) < target:
            d.drive(v_cmd, 0.0)
            time.sleep(0.05)
            l, r = d.read_feedback()
            now = time.time()
            sl.append((now - t0, l))
            sr.append((now - t0, r))
            dt, t_prev = now - t_prev, now
            v = (spd_to_ms(l, a.radius) + spd_to_ms(r, a.radius)) / 2.0
            dist += v * dt
            if now - t0 > 30:
                print('  [중단] 30초 초과 — 바퀴가 도는지 확인 (tools/motor_diag.py)')
                break

        dist_stop = dist
        t_stop = time.time()
        print('>>> 목표 도달 -> 정지 (주행 %.2fs, odom %.1f cm)'
              % (t_stop - t0, dist_stop * 100))

        # 정지 구간에서도 정지 명령을 재전송해야 피드백이 살아있다.
        v_last = None
        while time.time() - t_stop < a.settle:
            d.drive(0.0, 0.0)
            time.sleep(0.05)
            l, r = d.read_feedback()
            now = time.time()
            dt, t_prev = now - t_prev, now
            v_last = (spd_to_ms(l, a.radius) + spd_to_ms(r, a.radius)) / 2.0
            dist += v_last * dt
    finally:
        d.stop()
        d.close()

    print('')
    print('=' * 62)
    print(' 정지 명령 시점 odom : %7.1f cm' % (dist_stop * 100))
    print(' 관성 주행분         : %7.1f cm' % ((dist - dist_stop) * 100))
    print(' odom 총 이동거리    : %7.1f cm' % (dist * 100))
    print(' 정지 후 잔여 속도   : %7.1f cm/s   (0 에 가까워야 정상)' % ((v_last or 0) * 100))
    print(' 총 소요 시간        : %7.2f s' % (time.time() - t0))
    print('=' * 62)
    print(' 떨림 — spd 산포')
    print('  [정상상태 = 출발 %.1fs 이후]' % SETTLE_AFTER)
    print('   왼쪽(L) : %s' % stats(sl, SETTLE_AFTER))
    print('   오른쪽(R): %s' % stats(sr, SETTLE_AFTER))
    print('   기준: 0.20 m/s 에서 변동 2%, 0.05 m/s 에서 28% (2026-09-16 실측)')
    print('=' * 62)

    if a.measured > 0:
        k = a.measured / abs(dist) if abs(dist) > 1e-6 else 0.0
        print('')
        print(' 실측 거리     : %6.1f cm' % (a.measured * 100))
        print(' 보정계수      : %.4f' % k)
        print(' wheel_radius  : %.5f -> %.5f' % (a.radius, a.radius * k))
        print('')
        print(' 반영 방법:')
        print('   ros2 run relayrobot_description real_robot_driver_260519 \\')
        print('     --ros-args -p wheel_radius:=%.5f' % (a.radius * k))
    else:
        print('')
        print(' 줄자로 실제 거리를 재서 --measured 0.32 처럼 다시 주면 보정계수를 계산한다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
