"""/odom 을 구독해서 위치와 방향을 터미널에 찍는 최소 예제.

ROS 2 구독자가 어떻게 생겼는지 보려면 이 파일이 제일 짧다.

    ros2 run relayrobot_description odom_listener

[2026-09-22] relayrobot_driver 의 odom_sub(odom_subscriber.py)와 하나로 합쳤다.
두 노드가 하는 일이 완전히 같았다 — 둘 다 /odom 을 구독해 x, y, yaw 를 출력.
합치면서 odom_sub 쪽의 장점 두 개를 가져왔다:
  - 출력을 print 가 아니라 get_logger() 로. print 는 ROS 로그에 안 남고,
    launch 로 여러 노드를 띄우면 출력이 섞여 순서가 뒤엉킨다.
  - destroy_node() 를 finally 안으로. 예외로 빠져나갈 때도 정리된다.
구버전은 old_file/src/relayrobot_driver/ 에 있다.
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry   # 오도메트리 메시지 형식


class OdomListener(Node):

    def __init__(self):
        # 노드 이름. ros2 node list 에 이 이름으로 뜬다.
        super().__init__('odom_listener')

        # [2026-09-22] '/odom' 그대로 맞다. IMU 없는 기본 구성에서는 드라이버가
        #   /odom 을 직접 낸다. use_imu:=true 면 /odom 은 EKF 출력이 되는데,
        #   그때도 이 노드는 그대로 동작한다 (이름이 같으므로).
        #   드라이버 원본을 보고 싶을 때만 '/odom_raw' 로 바꾼다.
        self.subscription = self.create_subscription(
            Odometry,                # 메시지 타입 (어떤 형식의 편지인지)
            '/odom',                 # 토픽 이름 — 실제 발행되는 이름과 같아야 한다
            self.listener_callback,  # 메시지가 올 때마다 불릴 함수
            10,                      # 큐 크기: 처리가 밀리면 10개까지만 쌓아둔다
        )

        self.get_logger().info('odom listener started (subscribing /odom)')

    def listener_callback(self, msg: Odometry):
        # 1. 위치
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # 2. 쿼터니언 -> yaw (z축 회전).
        #    2D 로봇이라 roll/pitch 는 버리고 yaw 만 쓴다.
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # 3. 출력
        self.get_logger().info(
            f'Pos: X={x:.2f}, Y={y:.2f} | Yaw: {math.degrees(yaw):.1f}deg ({yaw:.2f} rad)')


def main(args=None):
    rclpy.init(args=args)       # ROS 2 통신 기능을 켠다
    node = OdomListener()
    try:
        rclpy.spin(node)        # 메시지가 올 때까지 기다리며 콜백을 돌린다
    except KeyboardInterrupt:
        pass                    # Ctrl+C 면 조용히 종료
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
