import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import math

# 드라이버 파일
from .motor_drive_1 import MotorDriver 

class RealRobotDriver(Node):
    def __init__(self):
        super().__init__('real_robot_driver')

        # ROS 2 Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('wheel_radius', 0.05) # Aligning with URDF (diameter 0.1)
        self.declare_parameter('wheel_base', 0.165)  # Aligning with URDF (separation 0.165)

        port = self.get_parameter('port').get_parameter_value().string_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value

        # 1. 모터 드라이버 연결
        try:
            self.driver = MotorDriver(port=port) 
            self.get_logger().info(f"DDSM400 Motor Connected to {port} Successfully!")
        except Exception as e:
            self.get_logger().error(f"Motor connection failed: {e}")
            self.driver = None

        # 2. 로봇 상태 변수
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # 바퀴 회전 각도 누적 변수
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0

        # 3. Publisher & Subscriber
        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.br = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 4. Timer (10Hz)
        self.timer_period = 0.1 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Odom & Joint Publisher Started...")

    def cmd_vel_callback(self, msg):
        if self.driver:
            self.driver.drive(msg.linear.x, msg.angular.z)

    def timer_callback(self):
        if not self.driver:
            return

        # [중요] 모든 메시지에 사용할 공통 시간 딱 한 번만 생성
        now = self.get_clock().now()
        current_time_msg = now.to_msg()

        # 1. Driver에서 RPM 피드백 읽기
        rpm_L, rpm_R = self.driver.read_feedback()

        # 1-1 터미널 속도 확인 
        self.get_logger().info(f"MOTORS -> L: {rpm_L}, R: {rpm_R}")

        # 2. 속도 변환 (dt 계산)
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        rpm_L = -rpm_L

        # RPM -> m/s // 실제로 100 을주면 10 rpm으로 깍으니까 600
        vl = (rpm_L / 600.0) * (2 * math.pi * self.wheel_radius)
        vr = (rpm_R / 600.0) * (2 * math.pi * self.wheel_radius)

        v = (vl + vr) / 2.0
        w = (vr - vl) / self.wheel_base

        # 3. 오도메트리 적분
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        
        # 4. 바퀴 각도 업데이트 (RViz에서 바퀴가 도는 모습)
        self.left_wheel_angle += (vl / self.wheel_radius) * dt
        self.right_wheel_angle += (vr / self.wheel_radius) * dt
        
        # 5. JointState 발행 (바퀴 회전 정보)
        joint_msg = JointState()
        joint_msg.header.stamp = current_time_msg
        joint_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_msg.position = [self.left_wheel_angle, self.right_wheel_angle]
        self.joint_pub.publish(joint_msg)
    
        # 6. Odom & TF 발행 (위치 정보 - 생성해둔 시간 전달)
        self.publish_odom(v, w, current_time_msg)

    def publish_odom(self, v, w, stamp):
        # 1. 쿼터니언 변환
        q = quaternion_from_euler(0, 0, self.theta)
        
        # 2. TF 변환 정보 (t)를 "먼저" 정의합니다. // 260204_etf값으로 했기때문에 나중에 패스 
        """
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # 3. 정의된 TF를 "먼저" 쏩니다.
        self.br.sendTransform(t)
        
        # 여기는 relarobyt_descritipn에 confing 저장한 ekf.yaml으로 그렇게 오도메트르릴 업데이트는하는거야 
        """
        # 4. Odometry 메시지를 설정합니다.
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        
        # 공분산 설정
        odom.pose.covariance = [1e-9] * 36
        odom.twist.covariance = [1e-9] * 36

        # 5. 마지막으로 Odom을 발행합니다.
        self.odom_pub.publish(odom)

    def stop_robot(self):
        if self.driver:
            self.driver.stop()
            self.driver.close()

def main(args=None):
    rclpy.init(args=args)
    node = RealRobotDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
