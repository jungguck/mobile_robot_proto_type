import serial
import time
import struct

# 가능한 속도 후보 (매뉴얼은 38400이지만, 혹시 몰라 115200도 넣음)
BAUD_RATES = [38400, 115200]
# 테스트할 ID 범위 (1번부터 10번까지)
TEST_IDS = range(1, 11)

def crc8_maxim(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc

def scan_motors():
    print("🕵️‍♂️ 모터 수색을 시작합니다...")
    
    for baud in BAUD_RATES:
        print(f"\n[테스트 중] 통신 속도: {baud}")
        try:
            ser = serial.Serial('/dev/ttyUSB0', baud, timeout=0.05)
        except Exception as e:
            print(f"⚠️ 포트 열기 실패 ({baud}): {e}")
            continue

        for motor_id in TEST_IDS:
            # 1. 0x74 (정보 요청) 패킷 생성
            # [ID] [0x74] [0]... [CRC]
            packet = bytearray(9)
            packet[0] = motor_id
            packet[1] = 0x74
            packet.append(crc8_maxim(packet))
            
            # 2. 전송
            ser.reset_input_buffer()
            ser.write(packet)
            time.sleep(0.02) # 짧게 대기
            
            # 3. 응답 확인
            if ser.in_waiting > 0:
                # 데이터가 왔다면? 범인 검거!
                data = ser.read(ser.in_waiting)
                print(f"🎉 찾았다!!! -> 속도: {baud}, ID: {motor_id}")
                print(f"   응답 데이터(Hex): {data.hex().upper()}")
                ser.close()
                return # 찾았으니 종료
            
            # 못 찾았으면 점 찍기
            print(".", end="", flush=True)
            
        ser.close()
        
    print("\n\n❌ 모든 속도와 ID를 다 뒤졌지만 반응이 없습니다.")
    print("결론: 소프트웨어 문제가 아닙니다. '선 연결(A/B)'을 반대로 바꿔보세요.")

if __name__ == "__main__":
    scan_motors()

# import serial
# import time
# import struct

# # [중요] 매뉴얼에 적힌 대로 Baudrate 38400 설정
# ser = serial.Serial('/dev/ttyACM0', 38400, timeout=0.1)

# def crc8_maxim(data):
#     crc = 0
#     for byte in data:
#         crc ^= byte
#         for _ in range(8):
#             if crc & 1:
#                 crc = (crc >> 1) ^ 0x8C
#             else:
#                 crc >>= 1
#     return crc

# def read_encoder(motor_id):
#     # --- 1. 명령 패킷 만들기 (Protocol 2: Other feedback) ---
#     # [ID] [0x74] [0] [0] [0] [0] [0] [0] [0] [CRC]
#     packet = bytearray(9)
#     packet[0] = motor_id
#     packet[1] = 0x74  # 매뉴얼: 0x74가 정보 요청 커맨드
    
#     # CRC 추가
#     packet.append(crc8_maxim(packet))
    
#     # --- 2. 전송 및 수신 ---
#     ser.reset_input_buffer()
#     ser.write(packet)
    
#     # 응답 대기 (38400bps라 조금 넉넉히)
#     time.sleep(0.05) 
    
#     if ser.in_waiting >= 10: # 응답은 10바이트
#         data = ser.read(10)
        
#         # [중요] 매뉴얼 검증: 응답 코드가 0x75인지 확인
#         # 응답 패킷: [ID] [0x75] [Lap4] [Lap3] [Lap2] [Lap1] [PosH] [PosL] [Err] [CRC]
#         if data[0] == motor_id and data[1] == 0x75:
            
#             # --- 3. 데이터 해석 (Parsing) ---
#             # DATA[2]~[5]: Mileage laps (4바이트 정수, Big Endian)
#             # struct.unpack('>i') : Big Endian 정수
#             laps = struct.unpack('>i', data[2:6])[0]
            
#             # DATA[6]~[7]: Position value (2바이트 부호없는 정수, Big Endian)
#             # 0 ~ 32767 값
#             raw_pos = struct.unpack('>H', data[6:8])[0]
            
#             # --- 4. 최종 엔코더 값 계산 ---
#             # 한 바퀴는 32768 틱(Tick)이라고 가정 (0~32767이므로)
#             total_ticks = (laps * 32768) + raw_pos
            
#             print(f"🔄 바퀴 회전수: {laps}")
#             print(f"📏 현재 각도값: {raw_pos} (0~32767)")
#             print(f"🚀 [오도메트리용] 총 누적 엔코더: {total_ticks}")
#             print("-" * 30)
#         else:
#             print(f"⚠️ 이상한 응답: {data.hex().upper()}")

# if __name__ == "__main__":
#     print("엔코더 데이터 읽기 시작 (Ctrl+C로 종료)")
#     try:
#         while True:
#             # ID 1번에게 물어봄 (만약 반응 없으면 0이나 2로 바꿔보세요)
#             read_encoder(1) 
#             time.sleep(0.1)
#     except KeyboardInterrupt:
#         ser.close()