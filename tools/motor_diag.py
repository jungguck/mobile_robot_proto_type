#!/usr/bin/env python3
"""DDSM400 모터 진단 — "안 도는지 / 떨리는지" 를 추측 없이 숫자로 판정한다.

젯슨에서 실행한다.  python3 tools/motor_diag.py [--check|--sweep|--all]

  --check  좌우 모터를 하나씩 돌려 실제로 도는지 + 좌우 균형 (기본값)
  --sweep  속도별 정상상태 산포 — 떨림이 심한 속도 구간을 찾는다
  --all    둘 다

왜 이 스크립트가 필요한가
-------------------------
`spd` 피드백만 보면 오판한다. 이 보드는 명령을 받을 때만 회신하고, 드라이버는 새 프레임이
없으면 직전 값을 그대로 돌려준다. 그래서 **엔코더 절대위치**로 판정한다.

  ⚠️ `pos` 는 1회전(0~32767 = 360°)마다 되감긴다.
     반드시 `mil * 32768 + pos` 로 누적치를 만들어 비교할 것.
     (2026-09-16 에 이걸 몰라서 "2바퀴 돌았는데 안 움직임" 으로 오판했다)

  ⚠️ 반이중 RS485 다. 여러 모터에 연속으로 명령을 보내면 충돌해서 {"T":20010,"crc":0}
     이 돌아온다. 그래서 여기서는 **한 번에 모터 하나만** 다룬다.

  ⚠️ heartbeat 가 비활성(-1)이라 정지 명령을 안 보내면 모터가 영원히 돈다.
     모든 경로에서 finally 로 정지를 보장한다.

배경: docs/DEBUG_LOG_2026-09-16.md
"""
import argparse
import time

import serial

PORT = '/dev/motor'
NAME = {1: 'id1 오른쪽(R)', 2: 'id2 왼쪽(L)'}
DIR = {1: -1, 2: +1}          # 전진 부호. R 음수 / L 양수
R_WHEEL = 0.0325


def field(line, key):
    try:
        return int(line.split('"%s":' % key)[1].split(',')[0].split('}')[0])
    except (IndexError, ValueError):
        return None


class Board(object):
    def __init__(self, port=PORT):
        self.ser = serial.Serial(port, 115200, timeout=0)
        time.sleep(0.3)

    def cmd(self, s, dur=0.2):
        """명령 하나 전송 -> 응답 수신. 버스에 한 번에 하나만 올린다."""
        self.ser.write((s + '\n').encode())
        t0 = time.time()
        buf = b''
        while time.time() - t0 < dur:
            if self.ser.in_waiting:
                buf += self.ser.read(self.ser.in_waiting)
            time.sleep(0.01)
        txt = buf.decode(errors='replace')
        return [l.strip() for l in txt.splitlines() if l.strip()]

    def init(self):
        self.cmd('{"T":11002,"type":210}', 0.4)
        for mid in (1, 2):
            self.cmd('{"T":10012,"id":%d,"mode":2}' % mid, 0.2)

    def total_ticks(self, mid):
        """되감김 없는 누적 엔코더. mil*32768 + pos."""
        key = '"id":%d' % mid
        for _ in range(4):
            for line in self.cmd('{"T":10032,"id":%d}' % mid, 0.4):
                if '"pos"' in line and key in line:
                    mil, pos = field(line, 'mil'), field(line, 'pos')
                    if mil is not None and pos is not None:
                        return mil * 32768 + pos
            time.sleep(0.1)
        return None

    def spin(self, mid, cval, secs):
        """모터 하나를 secs 초 돌리고 (누적틱 변화, spd 샘플) 을 돌려준다."""
        before = self.total_ticks(mid)
        spds = []
        t0 = time.time()
        while time.time() - t0 < secs:
            for line in self.cmd('{"T":10010,"id":%d,"cmd":%d,"act":10}' % (mid, cval), 0.1):
                if '"spd"' in line:
                    v = field(line, 'spd')
                    if v is not None:
                        spds.append((time.time() - t0, v))
        self.cmd('{"T":10010,"id":%d,"cmd":0,"act":10}' % mid, 0.3)
        time.sleep(0.4)
        after = self.total_ticks(mid)
        revs = None
        if before is not None and after is not None:
            revs = (after - before) / 32768.0
        return revs, spds

    def stop_all(self):
        for mid in (1, 2):
            self.ser.write(('{"T":10010,"id":%d,"cmd":0,"act":10}' % mid).encode() + b'\n')
            time.sleep(0.1)
        time.sleep(0.2)
        self.ser.close()


def spread(vals):
    """평균, 표준편차, 변동률(%)."""
    if not vals:
        return None
    n = len(vals)
    mean = sum(vals) / n
    sd = (sum((x - mean) ** 2 for x in vals) / n) ** 0.5
    return mean, sd, (100 * sd / abs(mean)) if mean else 0.0


def do_check(b):
    print('===== 좌우 구동 확인 (cmd 294 = 0.1 m/s, 2초씩) =====')
    print('  로봇이 제자리에 있도록 좌우를 따로 돌립니다. 제자리 회전에 주의하세요.')
    got = {}
    for mid in (1, 2):
        revs, spds = b.spin(mid, DIR[mid] * 294, 2.0)
        vals = [v for (_, v) in spds]
        s = spread(vals)
        print('')
        print('  %s' % NAME[mid])
        if revs is None:
            print('    누적틱을 못 읽음 — 판정 불가')
        else:
            print('    회전량 %.3f 바퀴 (%.0f deg)   %s'
                  % (revs, revs * 360, '★ 정상' if abs(revs) > 0.05 else '⚠ 안 돎'))
            got[mid] = abs(revs)
        if s:
            print('    spd 평균 %.0f  표준편차 %.0f  변동 %.0f%%' % s)
        time.sleep(0.4)

    if len(got) == 2 and max(got.values()) > 0:
        diff = abs(got[1] - got[2]) / max(got.values()) * 100
        print('')
        print('  좌우 차이 %.1f%%  %s' % (diff, '✅ 균형 정상' if diff < 10 else '⚠ 불균형 의심'))


def do_sweep(b):
    print('')
    print('===== 속도별 정상상태 산포 (떨림 진단) =====')
    print('  방향을 번갈아 바꿔 제자리 근처에 머물게 합니다.')
    print('')
    print('  %-10s %-8s %s' % ('속도', 'cmd', '좌우 변동률 (작을수록 안정)'))
    sign = 1
    for cval in (147, 294, 441, 588):
        ms = cval / 600.0 * (2 * 3.14159265 * R_WHEEL)
        out = []
        for mid in (1, 2):
            _, spds = b.spin(mid, sign * DIR[mid] * cval, 1.5)
            # 출발 가속 구간을 빼야 한다. 안 그러면 산포가 부풀려진다.
            vals = [abs(v) for (t, v) in spds if t >= 0.6]
            s = spread(vals)
            out.append('%s %3.0f%%' % (NAME[mid][:3], s[2]) if s else '%s  --' % NAME[mid][:3])
            time.sleep(0.2)
        sign = -sign
        print('  %-10s %-8d %s' % ('%.2f m/s' % ms, cval, '   '.join(out)))
    print('')
    print('  기준: 0.05 m/s 에서 약 28%, 0.20 m/s 에서 약 2% 가 정상 (2026-09-16 실측)')
    print('  >>> 순항 속도는 0.15 m/s 이상으로 잡을 것.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true', help='좌우 구동 확인 (기본)')
    ap.add_argument('--sweep', action='store_true', help='속도별 떨림 산포')
    ap.add_argument('--all', action='store_true', help='둘 다')
    ap.add_argument('--port', default=PORT)
    a = ap.parse_args()
    if not (a.check or a.sweep or a.all):
        a.check = True

    b = Board(a.port)
    try:
        b.init()
        if a.check or a.all:
            do_check(b)
        if a.sweep or a.all:
            do_sweep(b)
    finally:
        b.stop_all()
        print('')
        print('정지 완료.')


if __name__ == '__main__':
    main()
