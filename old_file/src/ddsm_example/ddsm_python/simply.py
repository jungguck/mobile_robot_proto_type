import serial
import argparse
import threading
import sys
import time

# ==========================================
def read_serial():
    while True:
        try:
            if ser.in_waiting > 0:
                # 1. 일단 있는 데이터 싹 긁어옴
                data = ser.read(ser.in_waiting)
                
                # 2. 텍스트로 변환 시도
                try:
                    text = data.decode('utf-8')
                    print(f"\n[텍스트 수신] {text}", end='')
                except:
                    # 텍스트 아니면 16진수로 보여줌 (깨짐 방지)
                    print(f"\n[Hex 수신] {data.hex().upper()}")
                
                print("\n명령어 입력 >> ", end='', flush=True)
        except Exception as e:
            print(f"Error: {e}")
            break
        time.sleep(0.01)

def main():
    global ser
    parser = argparse.ArgumentParser()
    # 기본값 ttyACM0 설정 (매번 치기 귀찮으니까)
    parser.add_argument('port', nargs='?', default='/dev/ttyACM0')
    args = parser.parse_args()

    print(f"🔌 {args.port} 연결 중...")

    try:
        ser = serial.Serial(args.port, 115200, timeout=0.1)
        print("✅ 연결 성공! 명령어를 입력하세요.")
        print("예시: {\"T\":10010,\"id\":2,\"cmd\":200,\"act\":10}")
        print("------------------------------------------------")
    except Exception as e:
        print(f"❌ 연결 실패: {e}")
        return

    # 수신 쓰레드 시작
    t = threading.Thread(target=read_serial)
    t.daemon = True
    t.start()

    try:
        while True:
            # 입력 대기
            user_input = input("명령어 입력 >> ")
            if user_input.strip():
                # 입력값 + 엔터(\n) 전송
                ser.write((user_input + '\n').encode())
    except KeyboardInterrupt:
        print("\n종료합니다.")
    finally:
        if 'ser' in globals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()