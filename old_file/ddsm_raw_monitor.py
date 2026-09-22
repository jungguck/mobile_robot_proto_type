#!/usr/bin/env python3
"""DDSM 보드 raw 응답 모니터 — "연결됐는데 토크가 없다" 진단용.

보드가 시리얼로 *응답을 하는지*, 속도 명령에 *전류(cur)/속도(spd)* 가 변하는지 raw 로 본다.
판단:
  - 아무 바이트도 안 옴            -> 보드 응답 없음 (포트/전원/보드 시리얼스위치/배선)
  - 응답은 오는데 spd/cur 0 고정   -> 모터 전원 미공급 or 모터 고장 (토크 안 남)
  - spd 가 변함                    -> 통신/구동 정상

[주의] launch 가 /dev/motor 를 점유 중이면 Ctrl+C 로 끄고 실행.

사용:
  python3 ddsm_raw_monitor.py                  # /dev/motor, type210, id1&2, cmd300(=30rpm)
  python3 ddsm_raw_monitor.py --type 115
  python3 ddsm_raw_monitor.py --ids 1 --cmd 500
"""
import argparse
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/motor')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--type', type=int, default=210)
    ap.add_argument('--ids', default='1,2')
    ap.add_argument('--cmd', type=int, default=300, help='속도명령 0.1rpm단위 (300=30rpm)')
    a = ap.parse_args()
    ids = [int(x) for x in a.ids.split(',') if x.strip()]

    print(f"[open] {a.port} @ {a.baud}")
    ser = serial.Serial(a.port, a.baud, timeout=0)
    time.sleep(0.3)

    def w(s):
        ser.write((s + '\n').encode())
        print("  >>", s)

    print(f"[init] type {a.type}, mode 2")
    w(f'{{"T":11002,"type":{a.type}}}')
    time.sleep(0.1)
    for i in ids:
        w(f'{{"T":10012,"id":{i},"mode":2}}')
        time.sleep(0.1)

    print("\n[run] 속도 명령 보내면서 raw 응답 모니터 (Ctrl+C 종료)\n")
    got_any = False
    t0 = time.time()
    try:
        while True:
            for i in ids:
                w(f'{{"T":10010,"id":{i},"cmd":{a.cmd},"act":10}}')
            time.sleep(0.05)
            if ser.in_waiting > 0:
                got_any = True
                raw = ser.read(ser.in_waiting)
                print("  RAW:", raw)
            else:
                print("  (응답 없음)")
            time.sleep(0.2)
            # 10초 후 한 번 진단 요약
            if not got_any and time.time() - t0 > 10:
                print("\n>>> 10초간 보드 응답 0 바이트. 포트/전원/보드 serial-switch(ESP32 위치)/배선 확인 필요.\n")
                t0 = time.time()
    except KeyboardInterrupt:
        print("\n[stop] 모터 정지 & 종료")
        for i in ids:
            ser.write(f'{{"T":10000,"id":{i}}}\n'.encode())
        ser.close()


if __name__ == '__main__':
    main()
