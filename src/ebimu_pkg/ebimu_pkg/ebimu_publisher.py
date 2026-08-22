import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import serial
import math
import time

# 노드 시작 직후 정지 상태에서 이 시간 동안 바이어스 수집
CALIB_SECONDS = 10


def euler_to_quaternion(roll, pitch, yaw):
    """오일러각(rad) → 쿼터니언 변환. ROS Imu 메시지는 쿼터니언만 받음."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = cy * sp * cr + sy * cp * sr
    q.z = sy * cp * cr - cy * sp * sr
    return q


class EbimuPublisher(Node):
    """
    역할: EBIMU9DOFV5 시리얼 데이터 → sensor_msgs/Imu 토픽 변환

    발행 토픽:
      /ebimu_data (sensor_msgs/Imu) → EKF가 구독해서 odom_raw와 융합

    시리얼 포맷 (센서 설정에 따라 둘 중 하나):
      *ROLL,PITCH,YAW,ACCX,ACCY,ACCZ,GYROX,GYROY,GYROZ   (9개)
      *ROLL,PITCH,YAW                                     (3개)
    2026-08-22 현재 이 센서는 3개만 출력한다.
    """

    def __init__(self):
        super().__init__('ebimu_publisher')

        self.declare_parameter('port',     '/dev/ttyimu')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')

        port          = self.get_parameter('port').get_parameter_value().string_value
        baudrate      = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        qos_profile    = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Imu, 'ebimu_data', qos_profile)

        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
            self.get_logger().info(f'Serial connected to {port}')
        except Exception as e:
            self.get_logger().error(f'Serial Error: {e}')
            exit(1)

        # 정지 상태 평균 = 바이어스 (센서 자체 오프셋)
        self.bias_accx  = 0.0
        self.bias_accy  = 0.0
        self.bias_gyroz = 0.0

        # 센서가 9개 필드(가속도/자이로 포함)를 주는지 여부. _calibrate 에서 확정.
        self.has_full_imu = True

        self.get_logger().info(f'Calibrating for {CALIB_SECONDS}s — IMU를 평평하게 고정하세요...')
        self._calibrate()
        self.get_logger().info(
            f'Calibration done! ACC bias: x={self.bias_accx:.4f} y={self.bias_accy:.4f} | GYRO bias z={self.bias_gyroz:.5f}'
        )

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz

    def _read_line(self):
        """시리얼 1줄 읽어서 float 리스트 반환. 파싱 실패 시 None.

        EBIMU 출력 포맷은 센서 설정에 따라 두 가지다.
          9개: *ROLL,PITCH,YAW,ACCX,ACCY,ACCZ,GYROX,GYROY,GYROZ
          3개: *ROLL,PITCH,YAW                (자세만 출력하도록 설정된 경우)

        2026-08-22 현재 이 센서는 3개만 내보낸다. 예전 코드는 9개가 아니면
        전부 버려서 /ebimu_data 가 한 번도 발행되지 않았다.
        3개짜리도 받아들이고, 없는 값(가속도/자이로)은 EKF 설정에서 끈다.
        """
        raw = self.ser.readline().decode('utf-8', errors='ignore').replace('\r', '').replace('\n', '').strip()
        if not raw.startswith('*'):
            return None
        words = raw.replace('*', '').split(',')
        if len(words) not in (3, 9):
            return None
        try:
            return [float(v) for v in words]
        except ValueError:
            return None

    def _calibrate(self):
        """CALIB_SECONDS 동안 정지 상태 샘플 수집 → ACC/GYRO 바이어스 계산.

        센서가 자세(3개)만 내보내는 설정이면 보정할 가속도/자이로가 없으므로
        바이어스는 0으로 두고 즉시 끝낸다. (10초를 헛되이 기다리지 않는다)
        """
        samples = []
        start = time.time()
        while time.time() - start < CALIB_SECONDS:
            d = self._read_line()
            if d:
                samples.append(d)
                if len(d) == 3:
                    # 자세만 나오는 센서 — 보정할 항목이 없다
                    self.has_full_imu = False
                    self.get_logger().info(
                        'IMU가 자세(roll/pitch/yaw)만 출력 — 가속도/자이로 보정 생략'
                    )
                    return

        if not samples:
            self.get_logger().warn('Calibration failed: no samples received')
            return

        n = len(samples)
        self.bias_accx  = sum(s[3] for s in samples) / n
        self.bias_accy  = sum(s[4] for s in samples) / n
        self.bias_gyroz = sum(s[8] for s in samples) / n

    def timer_callback(self):
        if self.ser.in_waiting == 0:
            return
        try:
            d = self._read_line()
            if d is None:
                return

            # EBIMU는 0~360° 출력 → EKF가 요구하는 -180~+180° 로 변환
            yaw_deg = d[2] - 360 if d[2] > 180 else d[2]

            msg = Imu()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            # 절대 방향값 (EKF가 yaw 드리프트 보정에 사용)
            roll  = math.radians(d[0])
            pitch = math.radians(d[1])
            yaw   = math.radians(yaw_deg)
            msg.orientation = euler_to_quaternion(roll, pitch, yaw)
            # covariance: EKF에게 "이 방향값 얼마나 믿어" 전달 (0이면 EKF 오작동)
            msg.orientation_covariance = [
                0.0025, 0.0,    0.0,
                0.0,    0.0025, 0.0,
                0.0,    0.0,    0.0025,
            ]

            if len(d) == 9:
                # 바이어스 제거 후 G → m/s² 변환
                msg.linear_acceleration.x = (d[3] - self.bias_accx) * 9.80665
                msg.linear_acceleration.y = (d[4] - self.bias_accy) * 9.80665
                msg.linear_acceleration.z = d[5] * 9.80665  # z축은 중력 포함 그대로
                msg.linear_acceleration_covariance = [
                    0.04, 0.0,  0.0,
                    0.0,  0.04, 0.0,
                    0.0,  0.0,  0.04,
                ]

                # 바이어스 제거 후 deg/s → rad/s 변환
                msg.angular_velocity.x = math.radians(d[6])
                msg.angular_velocity.y = math.radians(d[7])
                msg.angular_velocity.z = math.radians(d[8] - self.bias_gyroz)
                msg.angular_velocity_covariance = [
                    0.001, 0.0,   0.0,
                    0.0,   0.001, 0.0,
                    0.0,   0.0,   0.001,
                ]
            else:
                # 자세만 나오는 센서. 가속도/자이로는 "없음" 으로 표시한다.
                # ROS 규약: covariance[0] = -1 이면 해당 항목 미제공.
                # EKF 설정(imu0_config)에서도 이 항목들을 꺼둬야 한다.
                msg.linear_acceleration_covariance[0] = -1.0
                msg.angular_velocity_covariance[0] = -1.0

            self.publisher.publish(msg)

        except Exception as e:
            # 예전에는 조용히 삼켜서 "왜 토픽이 안 나오지" 를 진단할 수 없었다.
            self.get_logger().warn(f'IMU parse/publish failed: {e}', once=True)


def main(args=None):
    rclpy.init(args=args)
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
