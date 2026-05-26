import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import serial
import math

def euler_to_quaternion(roll, pitch, yaw):
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
    def __init__(self):
        super().__init__('ebimu_publisher')
        
        # ROS 2 Parameters
        self.declare_parameter('port', '/dev/ttyimu')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Imu, 'ebimu_data', qos_profile)
        
        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
            self.get_logger().info(f"✅ Serial connected to {port}")
        except Exception as e:
            self.get_logger().error(f"❌ Serial Error: {e}")
            exit(1)

        timer_period = 0.02 # 50Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            try:
                ser_data = self.ser.readline()
                decoded_data = ser_data.decode('utf-8', errors='ignore').strip()
                
                if decoded_data:
                    # EBIMU format: *ROLL,PITCH,YAW,ACCX,ACCY,ACCZ,GYROX,GYROY,GYROZ
                    clean_msg = decoded_data.replace('*', '')
                    words = clean_msg.split(',')
                    data = [float(val) for val in words if val.strip()]
                    
                    if len(data) >= 3:
                        msg = Imu()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = self.frame_id
                        
                        # Euler to Quaternion (Degrees to Radians conversion)
                        roll = math.radians(data[0])
                        pitch = math.radians(data[1])
                        yaw = math.radians(data[2])
                        msg.orientation = euler_to_quaternion(roll, pitch, yaw)
                        
                        # Add acceleration and gyro if available
                        if len(data) >= 9:
                            msg.linear_acceleration.x = data[3] * 9.80665 # G to m/s^2
                            msg.linear_acceleration.y = data[4] * 9.80665
                            msg.linear_acceleration.z = data[5] * 9.80665
                            msg.angular_velocity.x = math.radians(data[6]) # deg/s to rad/s
                            msg.angular_velocity.y = math.radians(data[7])
                            msg.angular_velocity.z = math.radians(data[8])

                        self.publisher.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Read Error: {e}")

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
