import serial
import json
import time
import math

class MotorDriver:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        # 1. 초기화
        self.MOTOR_ID_L = 2
        self.MOTOR_ID_R = 1
        self.wheel_radius = 0.035  # 7cm 지름 -> 반지름 0.035m
        self.wheel_base = 0.2      # 20cm
        
        self.current_rpm_L = 0
        self.current_rpm_R = 0

        print(f"Connecting to motor driver...")

        # 2. 시리얼 연결
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            print("Serial connected successfully.")
        except Exception as e:
            print(f"Failed to connect serial: {e}")
            self.ser = None

        if self.ser:
            self.initialize_motors()

    # 26.01.20  each motor setting has to be loading time /  
    def initialize_motors(self):
        print("Setting motors...")
        # 초기화 명령 등
        self.send_json({"T": 11002, "id": self.MOTOR_ID_L})
        time.sleep(0.01)
        self.send_json({"T": 11002, "id": self.MOTOR_ID_R})
        time.sleep(0.01)
        self.send_json({"T": 10012, "id": self.MOTOR_ID_L, "mode": 2})
        time.sleep(0.01)
        self.send_json({"T": 10012, "id": self.MOTOR_ID_R, "mode": 2})
        print("Motors initialized.")

    def calculate_rpms(self, v, w):
        # Differential Drive Kinematics
        v_left = v - (w * self.wheel_base / 2)
        v_right = v + (w * self.wheel_base / 2)

        # 대소문자 수정: self.WHEEL_RADIUS -> self.wheel_radius
        rpm_left = (v_left / (2 * math.pi * self.wheel_radius)) * 60 
        rpm_right = (v_right / (2 * math.pi * self.wheel_radius)) * 60 

        # print(f"rpm_left :{rpm_right} , rpm_right :{rpm_right}") # rpm 확인 
        return int(rpm_left), int(rpm_right)

    def send_json(self, data):
        if self.ser and self.ser.is_open:
            try:
                msg = json.dumps(data) + '\n'
                self.ser.write(msg.encode())
                time.sleep(0.001) 
            except Exception as e:
                print(f"Serial write failed: {e}")

     

    # 26.01.19 갑자기 모터가 동일하게 속도가 안나서 문제 발생 
    def drive(self, v, w):
        rpm_L, rpm_R = self.calculate_rpms(v, w)
        print(f" L : {rpm_L}, R: {rpm_R}")
        
    

        # 오른쪽
        cmd_r = {"T": 10010, "id": self.MOTOR_ID_R, "cmd": rpm_R, "act": 10}
        self.send_json(cmd_r)

        time.sleep(0.05)


        # 왼쪽
        cmd_l = {"T": 10010, "id": self.MOTOR_ID_L, "cmd": -rpm_L, "act": 10}

        self.send_json(cmd_l)





    def stop(self):
        self.drive(0, 0)
        print("\nStopped.")

    def close(self):
        if self.ser:
            self.ser.close()