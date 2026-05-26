import serial
import time

# 1. 시리얼 설정 (포트는 본인 환경에 맞게)
# timeout을 0으로 설정해서 '기다리지 말고 있는 거 다 내놔' 모드로 바꿉니다.
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0)

def raw_feedback_test():
    print("--- DDSM400 Raw Feedback Monitor ---")
    
    # [1] 모터 활성화 (딱 한번 전송)
    # JSON 라이브러리 안 쓰고 직접 문자열로 쏩니다 (이게 더 확실함)
    init_cmd = b'{"T":11002,"id":1}\n'
    mode_cmd = b'{"T":10012,"id":1,"mode":2}\n'
    
    ser.write(init_cmd)
    time.sleep(0.1)
    ser.write(mode_cmd)
    time.sleep(0.1)
    
    print("초기화 명령 전송 완료. 데이터 수신 대기 중...")
    
    # [2] 상태 확인 루프
    # 속도 0 명령 (act:0은 토크 해제일 수 있으니 act:10으로 확실하게 잡음)
    check_cmd = b'{"T":10010,"id":1,"cmd":0,"act":10}\n'

    while True:
        # 1. 버퍼 비우기 (찌꺼기 제거)
        ser.reset_input_buffer()
        
        # 2. 명령 전송
        ser.write(check_cmd)
        
        # 3. 아주 짧게 대기 (모터가 응답할 시간 0.02초)
        time.sleep(0.02)
        
        # 4. 들어온 게 있나 확인
        if ser.in_waiting > 0:
            # 5. 있는 대로 다 읽어옴 (byte 형태)
            raw_data = ser.read(ser.in_waiting)
            
            # 6. 그냥 바이트 그대로 출력 (깨지든 말든 일단 봄)
            print(f"RAW DATA: {raw_data}")
            
            # 7. 혹시 글자로 변환 되면 변환해서 출력
            try:
                print(f"DECODED : {raw_data.decode('utf-8')}")
            except:
                pass
            print("-" * 20)
            
        else:
            # 데이터가 없으면 점(.)만 찍어서 프로그램이 살아있는지 확인
            print(".", end="", flush=True)
            
        # 너무 빠르면 보기 힘드니 약간 대기
        time.sleep(0.1)

if __name__ == "__main__":
    try:
        raw_feedback_test()
    except KeyboardInterrupt:
        ser.close()
        print("\n종료합니다.")