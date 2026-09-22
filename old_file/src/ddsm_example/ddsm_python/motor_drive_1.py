import serial
import json
import time
import math

class MotorDriver:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.MOTOR_ID_L = 2
        self.MOTOR_ID_R = 1
        self.wheel_radius = 0.035  
        self.wheel_base = 0.2     
        
        # 피드백 저장을 위한 변수
        self.current_rpm_L = 0
        self.current_rpm_R = 0

        print(f"Connecting to motor driver...")
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.05) # 타임아웃 짧게
            print("Serial connected successfully.")
        except Exception as e:
            print(f"Failed to connect serial: {e}")
            self.ser = None

        if self.ser:
            self.initialize_motors()

    def initialize_motors(self):
        print("Setting motors...")

        self.send_json({"T": 11002, "id": self.MOTOR_ID_L})
        time.sleep(0.01)
        self.send_json({"T": 11002, "id": self.MOTOR_ID_R})
        time.sleep(0.01)
        self.send_json({"T": 10012, "id": self.MOTOR_ID_L, "mode": 2})
        time.sleep(0.01)
        self.send_json({"T": 10012, "id": self.MOTOR_ID_R, "mode": 2})
        print("Motors initialized.")


    def calculate_rpms(self, v, w):
        v_left = v - (w * self.wheel_base / 2)
        v_right = v + (w * self.wheel_base / 2)
        rpm_left = (v_left / (2 * math.pi * self.wheel_radius)) * 60 
        rpm_right = (v_right / (2 * math.pi * self.wheel_radius)) * 60 
        return int(rpm_left), int(rpm_right)


    def send_json(self, data):
        if self.ser and self.ser.is_open:
            try:
                msg = json.dumps(data) + '\n'
                self.ser.write(msg.encode())
            except Exception as e:
                print(f"Serial write failed: {e}")


    # [수정됨] 읽기 기능 강화    

    def read_feedback(self):
        if not self.ser or not self.ser.is_open:
            return self.current_rpm_L, self.current_rpm_R

        # 버퍼에 쌓인 데이터가 있으면 모두 읽어서 최신 상태로 갱신
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line: continue
                print(f"Debug faw data : {line}")
                
                # JSON 파싱 시도
                try:
                    data = json.loads(line)
                except json.JSONDecodeError:
                    continue # JSON 깨지면 무시하고 다음 줄 읽음

                # "spd" 키가 피드백 데이터라고 하셨으므로 그대로 사용
                if "id" in data and "spd" in data: 
                    motor_id = data["id"]
                    rpm = data["spd"] 
                    
                    if motor_id == self.MOTOR_ID_L:
                        # 왼쪽 모터는 명령 보낼 때 (-)를 붙였으니, 
                        # 읽어올 때도 (-)를 붙여야 오도메트리상 '전진(+)'이 됨
                        self.current_rpm_L = rpm 
                        # print(f"Update L: {self.current_rpm_L}")

                    elif motor_id == self.MOTOR_ID_R:
                        self.current_rpm_R = rpm
                        # print(f"Update R: {self.current_rpm_R}")
                        
            except Exception as e:
                print(f"Read Error: {e}")
        
        # 가장 최신으로 업데이트된 RPM 값을 리턴
        return self.current_rpm_L, self.current_rpm_R

    def drive(self, v, w):
        rpm_L, rpm_R = self.calculate_rpms(v, w)
        
        # 명령 전송
        cmd_r = {"T": 10010, "id": self.MOTOR_ID_R, "cmd": rpm_R, "act": 2}
        self.send_json(cmd_r)

        time.sleep(0.05)

        cmd_l = {"T": 10010, "id": self.MOTOR_ID_L, "cmd": -rpm_L, "act": 2}
        self.send_json(cmd_l)
        
        # 명령 보낸 직후에 혹시 쌓인 응답 읽기
        self.read_feedback()

    def stop(self):
        self.drive(0, 0)
        print("\nStopped.")

    def close(self):
        if self.ser:
            self.ser.close()