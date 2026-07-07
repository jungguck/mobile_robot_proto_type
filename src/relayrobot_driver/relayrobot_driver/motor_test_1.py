"""
모터 단독 테스트 (motor_drive_1.py 의 drive() 로직과 동일)

전제 조건: 두 모터가 각각 ID=1(R), ID=2(L) 로 설정돼 있어야 함.
  → ID 확인/설정은 motor_id_check.py 참고.

펌웨어 명령 (json_cmd.h):
  T:10012  mode 변경  (2 = speed loop)
  T:10010  속도 제어  (cmd = 목표 RPM, DDSM115 기준)
  T:10000  정지
"""
import serial
import json
import time

PORT = '/dev/motor'
BAUDRATE = 115200

MOTOR_ID_L = 2
MOTOR_ID_R = 1

CMD = 100          # 목표 RPM (DDSM115는 cmd 값 = RPM)


def send_json(ser, data):
    msg = json.dumps(data) + '\n'
    ser.write(msg.encode())
    print(f"  TX: {msg.strip()}")


def read_all(ser):
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(f"  RX: {line}")


def set_speed(ser, motor_id, cmd):
    send_json(ser, {"T": 10010, "id": motor_id, "cmd": cmd, "act": 10})


def spin(ser, motor_id, cmd, seconds, label):
    print(f"\n[{label}] id={motor_id}, cmd={cmd}  ({seconds}초)")
    set_speed(ser, motor_id, cmd)
    end = time.time() + seconds
    while time.time() < end:
        read_all(ser)
        time.sleep(0.1)
    set_speed(ser, motor_id, 0)   # 정지
    time.sleep(0.3)
    read_all(ser)


def main():
    print(f"Connecting to {PORT} ...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
    time.sleep(0.5)
    print("Connected.\n")

    # 1. 모터 Enable (HAT(B): T:11002, id 별로 활성화 필수)
    print("[enable] 모터 활성화")
    send_json(ser, {"T": 11002, "id": MOTOR_ID_L})
    time.sleep(0.05)
    send_json(ser, {"T": 11002, "id": MOTOR_ID_R})
    time.sleep(0.05)

    # 2. speed loop 모드로 전환 (양쪽)
    print("[mode] speed loop (2) 설정")
    send_json(ser, {"T": 10012, "id": MOTOR_ID_L, "mode": 2})
    time.sleep(0.05)
    send_json(ser, {"T": 10012, "id": MOTOR_ID_R, "mode": 2})
    time.sleep(0.05)

    # 2. 각 모터 단독 회전 → 어느 바퀴인지 눈으로 확인
    spin(ser, MOTOR_ID_R, CMD, 3, "R 단독")
    spin(ser, MOTOR_ID_L, CMD, 3, "L 단독")

    # 3. 전진 (motor_drive_1.py drive() 와 동일: R=+, L=-)
    print(f"\n[전진] R={CMD}, L={-CMD}  (3초)")
    set_speed(ser, MOTOR_ID_R, CMD)
    time.sleep(0.05)
    set_speed(ser, MOTOR_ID_L, -CMD)
    end = time.time() + 3.0
    while time.time() < end:
        read_all(ser)
        time.sleep(0.1)

    # 4. 정지
    print("\n[정지]")
    set_speed(ser, MOTOR_ID_R, 0)
    time.sleep(0.05)
    set_speed(ser, MOTOR_ID_L, 0)
    time.sleep(0.3)
    read_all(ser)

    ser.close()
    print("\nDone.")


if __name__ == '__main__':
    main()
