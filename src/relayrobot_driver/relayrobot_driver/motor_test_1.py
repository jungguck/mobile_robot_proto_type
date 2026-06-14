import serial
import json
import time

PORT = '/dev/ttyACM0'
BAUDRATE = 115200

MOTOR_ID_L = 2
MOTOR_ID_R = 1

# cmd 100 = 10 RPM  →  cmd 10 = 1 RPM
CMD = 100


def send_json(ser, data):
    msg = json.dumps(data) + '\n'
    ser.write(msg.encode())
    print(f"  TX: {msg.strip()}")


def read_all(ser):
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"  RX: {line}")
        except Exception as e:
            print(f"  Read error: {e}")


def main():
    print(f"Connecting to {PORT} ...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
    time.sleep(0.5)
    print("Connected.\n")

    # 1. Initialize both motors
    print("[1] Initialize motors")
    send_json(ser, {"T": 11002, "id": MOTOR_ID_L})
    time.sleep(0.05)
    send_json(ser, {"T": 11002, "id": MOTOR_ID_R})
    time.sleep(0.05)

    # 2. Set mode 2 (speed control)
    print("\n[2] Set mode 2")
    send_json(ser, {"T": 10012, "id": MOTOR_ID_L, "mode": 2})
    time.sleep(0.05)
    send_json(ser, {"T": 10012, "id": MOTOR_ID_R, "mode": 2})
    time.sleep(0.05)

    # 3. Drive: cmd=10 → ~1 RPM  (L is negated for forward direction)
    print(f"\n[3] Drive cmd={CMD}  (L: -{CMD}, R: {CMD})")
    send_json(ser, {"T": 10010, "id": MOTOR_ID_R, "cmd":  CMD, "act": 10})
    time.sleep(0.05)
    send_json(ser, {"T": 10010, "id": MOTOR_ID_L, "cmd": -CMD, "act": 10})

    # 4. Read feedback for 10 seconds
    print("\n[4] Reading feedback for 10 seconds ...")
    end = time.time() + 10.0
    while time.time() < end:
        read_all(ser)
        time.sleep(0.1)

    # 5. Stop
    print("\n[5] Stop")
    send_json(ser, {"T": 10010, "id": MOTOR_ID_R, "cmd": 0, "act": 10})
    time.sleep(0.05)
    send_json(ser, {"T": 10010, "id": MOTOR_ID_L, "cmd": 0, "act": 10})
    time.sleep(0.2)
    read_all(ser)

    ser.close()
    print("\nDone.")


if __name__ == '__main__':
    main()
