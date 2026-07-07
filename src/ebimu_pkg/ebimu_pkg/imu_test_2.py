# 260614 / 바이어스 캘리브레이션 + EKF에 필요한 필드만 출력

import serial
import time

PORT = '/dev/ttyimu'
BAUDRATE = 115200
CALIB_SECONDS = 10

# EBIMU format: *ROLL,PITCH,YAW,ACCX,ACCY,ACCZ,GYROX,GYROY,GYROZ
# EKF 필요 필드: YAW(2), ACCX(3), ACCY(4), GYROZ(8)


def read_line(ser):
    raw = ser.readline().decode('utf-8', errors='ignore').replace('\r', '').replace('\n', '').strip()
    if not raw.startswith('*'):
        return None
    words = raw.replace('*', '').split(',')
    if len(words) != 9:
        return None
    try:
        return [float(v) for v in words]
    except ValueError:
        return None


def main():
    print(f"Connecting to {PORT} ...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(0.5)

    # 1. 초기 10초 바이어스 캘리브레이션
    print(f"\n[Calibration] IMU 평평하게 고정하세요. {CALIB_SECONDS}초간 수집합니다...\n")
    samples = []
    start = time.time()

    while time.time() - start < CALIB_SECONDS:
        d = read_line(ser)
        if d:
            samples.append(d)
        elapsed = time.time() - start
        print(f"\r  수집 중... {elapsed:.1f}s / {CALIB_SECONDS}s  ({len(samples)} samples)", end='', flush=True)

    print(f"\n  완료! 총 {len(samples)}개 샘플 수집\n")

    if not samples:
        print("샘플 없음. 포트/연결 확인하세요.")
        return

    # 바이어스 = 정지 상태 평균값
    bias = [0.0] * 9
    for s in samples:
        for i in range(9):
            bias[i] += s[i]
    for i in range(9):
        bias[i] /= len(samples)

    print(f"  ACC  bias → x: {bias[3]:>8.4f}  y: {bias[4]:>8.4f}")
    print(f"  GYRO bias → z: {bias[8]:>8.5f}")
    print(f"  YAW  초기값: {bias[2]:>8.2f} deg\n")

    # 2. 실시간 출력 (EKF 필요 필드만)
    print(f"{'YAW(deg)':>12} | {'ACCX(G)':>10} {'ACCY(G)':>10} | {'GYROZ(deg/s)':>14}")
    print("-" * 60)

    while True:
        d = read_line(ser)
        if not d:
            continue

        yaw_raw = d[2] - 360 if d[2] > 180 else d[2]
        yaw   = yaw_raw - bias[2]  # 초기 yaw 기준으로 상대 각도
        accx  = d[3]  - bias[3]    # 바이어스 제거
        accy  = d[4]  - bias[4]
        gyroz = d[8]  - bias[8]

        print(f"{yaw:>12.2f} | {accx:>10.4f} {accy:>10.4f} | {gyroz:>14.5f}")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nDone.")
