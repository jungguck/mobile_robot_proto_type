import serial
import time

def calc_crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1: crc = (crc >> 1) ^ 0x8C
            else: crc >>= 1
    return crc

# [수정됨] 여기를 38400으로 꼭 바꾸세요!!!!
ser = serial.Serial('/dev/ttyACM0', 38400, timeout=0.1)
print("Connecting at 38400 (Fixed)...")

def send_blind(id, mode):
    data = [id, 0xA0, mode, 0, 0, 0, 0, 0, 0]
    crc = calc_crc8(data)
    packet = bytearray(data + [crc])
    ser.write(packet)
    print(f"Sent Enable Command to ID {id}")

try:
    while True:
        send_blind(1, 0x08) # ID 1번 깨우기
        send_blind(2, 0x08) # ID 2번 깨우기
        print(">>> 손으로 돌려보세요. (Ctrl+C 종료)")
        time.sleep(0.5)
        
except KeyboardInterrupt:
    ser.close()