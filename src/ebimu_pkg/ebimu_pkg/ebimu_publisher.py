import rclpy # ros2 파이썬 클라이언트 
from rclpy.node import Node # ros2 node 설정을 위한 클래스 
from rclpy.qos import QoSProfile # 통신 품질을 위한 클래스 
from std_msgs.msg import String
import serial # 시리얼 usb 
import time

class EbimuPublisher(Node):
    def __init__(self):

        # 노드 초기화 (ebimu_publisher)
        super().__init__('ebimu_publisher') 
        
        # QOS 설정 데이터 보간함 10 으로 설정
        qos_profile = QoSProfile(depth=10)

        # 퍼블리쉬 생성, string 메세지 내용 전달 'ebimu_data' 라는 토픽으이름 전송 
        self.publisher = self.create_publisher(String, 'ebimu_data', qos_profile)
        
        # --- [수정 포인트 1] 시리얼 설정 ---
        # 사용자 입력 받기
        # port_input = input("EBIMU Port Number (e.g. USB0): ")
        comport_num = '/dev/imu' 
        # baud_rate = input("Baudrate (e.g. 115200): ")
        baud_rate = '115200'
        
        try:
            self.ser = serial.Serial(port=comport_num, baudrate=int(baud_rate), timeout=0.1)
            print(f"✅ Serial connected to {comport_num}")
        except Exception as e:
            print(f"❌ Serial Error: {e}")
            exit(1) # 연결 실패시 프로그램 종료

        # --- [수정 포인트 2] 주기 조정 (0.02초 = 50Hz 권장) ---
        timer_period = 0.02 
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # --- [수정 포인트 3] 데이터가 있을 때만 읽기 (Non-blocking) ---
        if self.ser.in_waiting > 0:
            try:
                # 줄바꿈까지 읽고 디코딩 (에러 무시 옵션 추가)
                ser_data = self.ser.readline()
                decoded_data = ser_data.decode('utf-8', errors='ignore').strip()
                
                # 빈 데이터가 아니면 보냄
                if decoded_data:
                    msg = String()
                    msg.data = decoded_data
                    self.publisher.publish(msg)
                    # 확인용 로그 (잘 되면 주석 처리 하세요)
                    # print(f"Sent: {msg.data}") 
            except Exception as e:
                print(f"Read Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    print("Starting ebimu_publisher..")
    
    node = EbimuPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
