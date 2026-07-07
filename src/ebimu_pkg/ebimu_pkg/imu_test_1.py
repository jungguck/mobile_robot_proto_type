# 260614

import serial
import time

PORT = '/dev/ttyUSB0'
BAUDRATE = 115200

# EBIMU format: *ROLL,PITCH,YAW,ACCX,ACCY,ACCZ,GYROX,GYROY,GYROZ


def main():
    print(f"Connecting to {PORT} ...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(0.5)
    print("Connected. Reading IMU raw values (Ctrl+C to stop)\n")
    print(f"{'ROLL':>8} {'PITCH':>8} {'YAW':>8} | {'ACCX':>8} {'ACCY':>8} {'ACCZ':>8} | {'GYROX':>8} {'GYROY':>8} {'GYROZ':>8}")
    print("-" * 90)

    while True:
        line = ser.readline().decode('utf-8', errors='ignore').replace('\r', '').replace('\n', '').strip()
        if not line.startswith('*'):
            continue
        words = line.replace('*', '').split(',')
        if len(words) != 9:
            continue
        try:
            d = [float(v) for v in words]
            print(f"{d[0]:>8.2f} {d[1]:>8.2f} {d[2]:>8.2f} | {d[3]:>8.3f} {d[4]:>8.3f} {d[5]:>8.3f} | {d[6]:>8.4f} {d[7]:>8.4f} {d[8]:>8.4f}")
        except ValueError:
            continue


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nDone.")
