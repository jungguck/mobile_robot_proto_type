import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import serial

class EbimuPublisher(Node):
    def __init__(self, port, baud):
        super().__init__('ebimu_publisher')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(String, 'ebimu_data', qos_profile)
        
        # 시리얼 설정 (클래스 내부에서 관리)
        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=1.0)
            self.get_logger().info(f"Connected to {port} at {baud} bps")
        except Exception as e:
            self.get_logger().error(f"Serial port error: {e}")
            exit()

        # 타이머 주기를 0.01 (100Hz) 정도로 조정하는 것을 권장합니다.
        timer_period = 0.01 
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.ser.in_waiting > 0: # 읽을 데이터가 있을 때만 동작
            try:
                ser_data = self.ser.readline()
                msg = String()
                # decode 에러 방지를 위해 ignore 옵션 추가
                msg.data = ser_data.decode('utf-8', errors='ignore').strip()
                self.publisher.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Read error: {e}")

def main(args=None):
    # 사용자 입력은 rclpy 시작 전에 받거나 파라미터로 처리
    port_input = input("EBIMU Port: /dev/tty")
    baud_input = input("Baudrate: ")
    full_port = '/dev/tty' + port_input

    rclpy.init(args=args)
    node = EbimuPublisher(full_port, int(baud_input))

    try:
        print("Starting ebimu_publisher..")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()