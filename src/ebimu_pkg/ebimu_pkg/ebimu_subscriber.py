import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class EbimuSubscriber(Node):
    def __init__(self):
        super().__init__('ebimu_subscriber')
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            String, 
            'ebimu_data', 
            self.callback, 
            qos_profile)

        # 클래스 변수 초기화
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0
        self.calib_count = 0
        self.is_calibrated = False
        self.target_sample = 30

    # [수정] self를 매개변수로 받아야 클래스 변수를 쓸 수 있습니다.
    def parse_and_process(self, msg_data):
        try:
            # 1. * 제거 및 콤마 분리
            clean_msg = msg_data.replace('*', '')
            words = clean_msg.strip().split(',')
            
            # 2. 실수 변환
            temp_data = [float(val) for val in words if val.strip()]

            # 3. 데이터 처리 로직
            if len(temp_data) >= 3:
                current_roll = temp_data[0]
                current_pitch = temp_data[1]
                current_yaw = temp_data[2]

                # --- 보정 모드 ---
                if not self.is_calibrated:
                    if self.calib_count < self.target_sample:
                        self.offset_x += current_roll
                        self.offset_y += current_pitch
                        self.offset_z += current_yaw
                        self.calib_count += 1
                        print(f"🔄 0점 보정 중... ({self.calib_count}/{self.target_sample})")
                    else:
                        # 30개 다 모았으니 평균 계산
                        self.offset_x /= self.target_sample
                        self.offset_y /= self.target_sample
                        self.offset_z /= self.target_sample
                        self.is_calibrated = True
                        print(f"✅ 보정 완료! Offsets -> R:{self.offset_x:.2f}, P:{self.offset_y:.2f}, Y:{self.offset_z:.2f}")

                # --- 측정 모드 ---
                else:
                    final_roll = current_roll - self.offset_x
                    final_pitch = current_pitch - self.offset_y
                    final_yaw = current_yaw - self.offset_z
                    
                    print(f"Roll : {final_roll:>6.2f}, Pitch : {final_pitch:>6.2f}, Yaw : {final_yaw:>6.2f}")

        except Exception as e:
            print(f"❌ 파싱 에러: {e} | 데이터: {msg_data}")

    def callback(self, msg):
        # [중요] 클래스 안에 있는 함수를 부를 땐 반드시 self. 을 붙여야 합니다!
        self.parse_and_process(msg.data)

def main(args=None):
    rclpy.init(args=args)
    print("Starting ebimu_subscriber..")
    node = EbimuSubscriber() # 여기서 __init__ 실행됨

    try:
        rclpy.spin(node) # 여기서 callback 무한 대기 시작
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()