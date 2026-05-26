import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
import math

class EbimuSubscriber(Node):
    def __init__(self):
        super().__init__('ebimu_subscriber')
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Imu, 
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

    def quaternion_to_euler(self, q):
        # roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def callback(self, msg):
        roll, pitch, yaw = self.quaternion_to_euler(msg.orientation)
        
        # --- 보정 모드 ---
        if not self.is_calibrated:
            if self.calib_count < self.target_sample:
                self.offset_x += roll
                self.offset_y += pitch
                self.offset_z += yaw
                self.calib_count += 1
                print(f"🔄 0점 보정 중... ({self.calib_count}/{self.target_sample})")
            else:
                self.offset_x /= self.target_sample
                self.offset_y /= self.target_sample
                self.offset_z /= self.target_sample
                self.is_calibrated = True
                print(f"✅ 보정 완료! Offsets -> R:{self.offset_x:.2f}, P:{self.offset_y:.2f}, Y:{self.offset_z:.2f}")

        # --- 측정 모드 ---
        else:
            final_roll = roll - self.offset_x
            final_pitch = pitch - self.offset_y
            final_yaw = yaw - self.offset_z
            
            print(f"Roll : {final_roll:>6.2f}, Pitch : {final_pitch:>6.2f}, Yaw : {final_yaw:>6.2f}")

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