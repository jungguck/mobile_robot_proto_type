"""
모터 ID 확인 / 변경 도구

펌웨어 명령 (json_cmd.h 기준):
  T:10031  → 연결된 모터의 ID 조회 (반드시 모터 1개만 연결)
  T:10011  → 모터 ID 변경 (모터 1개만 연결, 전원 사이클당 1회만 가능)

⚠ 반드시 모터를 '한 개만' HAT에 연결한 상태에서 실행하세요.
   두 개 연결 상태에서 ID를 변경하면 둘 다 같은 ID로 바뀝니다.
"""
import serial
import json
import time
import sys

PORT = '/dev/motor'
BAUDRATE = 115200


def send_json(ser, data):
    msg = json.dumps(data) + '\n'
    ser.write(msg.encode())


def read_responses(ser, wait=0.5):
    responses = []
    deadline = time.time() + wait
    while time.time() < deadline:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                responses.append(line)
        else:
            time.sleep(0.01)
    return responses


def check_id(ser):
    """T:10031 로 현재 연결된 단일 모터의 ID 조회"""
    ser.reset_input_buffer()
    send_json(ser, {"T": 10031})
    responses = read_responses(ser, wait=0.6)
    found_id = None
    for r in responses:
        try:
            data = json.loads(r)
            if "id" in data:
                found_id = data["id"]
        except json.JSONDecodeError:
            pass
    return found_id, responses


def change_id(ser, new_id):
    """T:10011 로 현재 연결된 단일 모터의 ID 변경.

    DDSM 모터는 ID 변경 프레임을 5번 연속 받아야 저장됨
    (펌웨어 ddsm_change_id 의 for(i<5) 루프와 동일).
    """
    ser.reset_input_buffer()
    for i in range(5):
        send_json(ser, {"T": 10011, "id": new_id})
        time.sleep(0.05)
    return read_responses(ser, wait=0.6)


def main():
    print(f"Connecting to {PORT} ...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
    time.sleep(0.5)
    print("Connected.\n")

    print("=" * 55)
    print("  ⚠ 모터를 '한 개만' 연결한 상태인지 확인하세요!")
    print("=" * 55)

    # 현재 ID 조회
    found_id, raw = check_id(ser)
    print(f"\n[조회] T:10031 응답: {raw}")
    if found_id is not None:
        print(f"  → 현재 연결된 모터 ID = {found_id}")
    else:
        print("  → ID를 읽지 못함 (모터 미연결/전원 확인, 또는 2개 연결로 충돌)")

    # 인자로 새 ID를 주면 변경 모드
    if len(sys.argv) >= 2:
        new_id = int(sys.argv[1])
        print(f"\n[변경] 이 모터의 ID를 {new_id} 로 변경합니다 ...")
        resp = change_id(ser, new_id)
        print(f"  응답: {resp}")
        print("  ⚠ 전원 OFF → ON 후 다시 조회하면 반영됩니다.")
        print("    (ID 변경은 전원 사이클당 1회만 가능)")
    else:
        print("\n[안내] ID를 바꾸려면 새 ID를 인자로 주고 다시 실행:")
        print("    python3 motor_id_check.py 2      # 이 모터를 ID=2 로 변경")

    ser.close()


if __name__ == '__main__':
    main()
