import serial
import threading
import sys
import json
import time
import math

# 1. 로봇 통신 설정 

PORT = '/dev/ttyACM0'
BAUDRATE = 115200

MOTOR_ID_L = 1
MOTOR_ID_R = 2

WHEEL_RADIUS = 0.07
WHEEL_BASE = 0.02

# 전역 변수 
ser = None

#보낼 명령 
# CMD : 100 -> 10 RPM
# ACT : 각가속도

# 1 

def send_json(data):
	""" json data를 만들어서 시리얼로 보냄 ㅋ """

	global serial
	try: 
		if ser and ser.is_open:
			# 딕서너리 -> 문자열 -> 바이트 변환 -> 줄바꿈 
			msg = json.dumps(data) + '\n'
			ser.write(msg.endoe())

			time.sleep(0.01)
	except Exception as e:
		print("fail")


#  2. diffcontrol

def calculate_rpms(v,w):

	# diffenential_drive_kinematics

	v_left = v - (w * WHEEL_BASE / 2 )
	v_right = v + (w * WHEEL_BASE / 2)

	# m/s --> RPM trnasimisson (속도 / 원주 ) *60
	
	rpm_left = (v_left / (2 * math.pi * WHEEL_RADIUS)) * 60
	rpm_right = (v_right / (2 * math.pi * WHEEL_RADIUS)) * 60

	return int(rpm_left), int(rpm_right)

# init motors()

def init_motors():
	print("loading. ...")

	send_json({"T":11002, "id": MOTOR_ID_L})
	send_json({"T":11002, "id": MOTOR_ID_R})

	print("enable_setting")

	send_json({"T":10012, "id": MOTOR_ID_L, "mode" : 2})
	send_json({"T":10012, "id": MOTOR_ID_R, "mode" : 2})

	print("change_mode_speed")


def main():

	global ser
    parser = argparse.ArgumentParser()
	    # 기본값 ttyACM0 설정 (매번 치기 귀찮으니까)
    parser.add_argument('port', nargs='?', default='/dev/ttyACM0') #이렇게해야 ACM이 변경해도 터미널에서 바로 대응 가능 
    args = parser.parse_args()
    print(f"🔌 {args.port} 연결 중...")

	try: 
		ser = serial.Serial(args.port, BAUDRATE, timeout=0.1)
		print("connecting_success")

		init_motors()
		print("checkt_motor")

			# [시뮬레이션] 제어기에서 v, w가 이렇게 들어온다고 가정
            # 나중에는 이곳을 조이스틱이나 상위 제어기 변수로 연결하면 됨
            # -------------------------------------------------

         while True:

            target_v = 0.5   # 0.5 m/s 앞으로
            target_w = 0.0   # 회전 없음
            #1. RPM 계산 
            rpm_L, rpm_R = calculate_rpms(target_v,target_w)
            #2. 명령전송 
            send_motor_command(MOTOR_ID_L, rpm_L)
            time.sleep(0.00001)
            send_motor_command(MOTOR_ID_R, rpm_R)

     		 # 너무 빠르면 모터 드라이버가 못 받아먹음
            time.sleep(0.05)
            
            print(f"\r[제어] v={target_v}, w={target_w} -> L_RPM={rpm_L}, R_RPM={rpm_R}   ", end="")

    except KeyboardInterrupt:

    	send_motor_command(MOTOR_ID_L,0)
	   	send_motor_command(MOTOR_ID_R,0)
	   	print("\n stop")

	finally:

		ser.close()

if __name__ == "__main__":
	main()
