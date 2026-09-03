import serial
import json
import time
import math

class MotorDriver:
    def __init__(self, port='/dev/motor', baudrate=115200,
                 wheel_radius=0.0325, wheel_base=0.22):
        # 2026-08-22 실측 확인: id=1 -> 오른쪽 바퀴, id=2 -> 왼쪽 바퀴
        self.MOTOR_ID_L = 2
        self.MOTOR_ID_R = 1

        # 좌우 허브모터가 거울 대칭으로 장착돼 있어 같은 부호를 주면 반대로 돈다.
        # 2026-08-22 실측: R=+, L=- 로 주니 로봇이 후진 -> 규약을 뒤집는다.
        # "+ = 로봇 전진" 이 되도록 하드웨어 부호를 여기서 흡수한다.
        self.DIR_R = -1
        self.DIR_L = +1
        # 실측: 바퀴 지름 65mm, 양 바퀴 중심간 거리 220mm
        # 오도메트리 노드와 반드시 같은 값을 써야 하므로 노드에서 주입받음
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        
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

        # T:11002(CMD_TYPE) 는 "id" 가 아니라 "type" 을 받는다.
        # 펌웨어(ddsm_example.ino: set_ddsm_type)는 115 / 210 만 인정하며,
        # DDSM400 은 DDSM210 계열 프로토콜(cmd/spd 단위 = 0.1rpm)을 쓴다.
        # 전원 사이클당 1회 필요. 이걸 빼먹으면 보드가 DDSM115 로 동작해
        # 구동 명령에 응답조차 하지 않는다. (2026-08-22 확인)
        self.send_json({"T": 11002, "type": 210})
        time.sleep(0.05)
        self.send_json({"T": 10012, "id": self.MOTOR_ID_L, "mode": 2})
        time.sleep(0.01)
        self.send_json({"T": 10012, "id": self.MOTOR_ID_R, "mode": 2})
        time.sleep(0.01)
        print("Motors initialized.")


    def calculate_rpms(self, v, w):
        """m/s, rad/s -> 좌우 모터 cmd 값.

        반환 단위는 RPM 이 아니라 DDSM400 의 cmd 단위(0.1RPM)다.
        즉 cmd 100 = 10 RPM. (2026-08-22 실측: cmd 100 -> spd 105 회신)
        그래서 60(RPM→RPS) 이 아니라 ×10(0.1RPM 단위)까지 합쳐 600 을 곱한다.

        이 ×600 은 real_robot_driver_260519.py 의 rpm_scale(÷600) 과
        정확히 서로의 역연산이다. 한쪽만 바꾸면 명령과 계측이 어긋난다.
        (2026-09-03 확정값 — 조정하지 말 것)
        """
        v_left = v - (w * self.wheel_base / 2)
        v_right = v + (w * self.wheel_base / 2)
        cmd_left = (v_left / (2 * math.pi * self.wheel_radius)) * 600
        cmd_right = (v_right / (2 * math.pi * self.wheel_radius)) * 600
        return int(cmd_left), int(cmd_right)


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
                # 피드백 프레임 예: {"T":20010,"id":1,"typ":400,"spd":105,"crt":296,...}
                if "id" in data and "spd" in data:
                    motor_id = data["id"]
                    rpm = data["spd"]

                    # drive() 와 같은 부호 규약을 적용해 "+ = 로봇 전진" 으로 맞춘다.
                    # 하드웨어 장착 방향은 여기서 전부 흡수하므로,
                    # 이 함수를 쓰는 쪽은 부호를 다시 뒤집지 말 것.
                    if motor_id == self.MOTOR_ID_L:
                        self.current_rpm_L = self.DIR_L * rpm

                    elif motor_id == self.MOTOR_ID_R:
                        self.current_rpm_R = self.DIR_R * rpm
                        
            except Exception as e:
                print(f"Read Error: {e}")
        
        # 가장 최신으로 업데이트된 RPM 값을 리턴
        return self.current_rpm_L, self.current_rpm_R

    def drive(self, v, w):
        cmd_L, cmd_R = self.calculate_rpms(v, w)

        # DIR_* 로 하드웨어 장착 방향을 흡수 -> v>0 이면 로봇이 전진한다.
        self.send_json({"T": 10010, "id": self.MOTOR_ID_R,
                        "cmd": int(self.DIR_R * cmd_R), "act": 10})

        time.sleep(0.01)

        self.send_json({"T": 10010, "id": self.MOTOR_ID_L,
                        "cmd": int(self.DIR_L * cmd_L), "act": 10})
        
        # 명령 보낸 직후에 혹시 쌓인 응답 읽기
        self.read_feedback()

    def stop(self):
        self.drive(0, 0)
        print("\nStopped.")

    def close(self):
        if self.ser:
            self.ser.close()