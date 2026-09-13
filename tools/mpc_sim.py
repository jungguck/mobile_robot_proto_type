#!/usr/bin/env python3
"""
mpc_sim.py — 로봇 없이 A* + Tube-MPC 를 끝까지 돌려보는 PC 전용 리허설 노드

[무엇을 하나]
  젯슨도 USB 장치도 없이, PC 한 대에서 자율주행 체인을 닫힌 루프로 돌린다.

    ① 가짜 /map 발행         — 20×20m, res 0.05, 테두리 벽 + 중앙 장애물 + 가림벽(틈 있음)
    ② /cmd_vel 구독          — 차동 구동 기구학으로 적분 (진짜 로봇 대신)
    ③ TF map→odom→base_link — SLAM + EKF 가 하는 일을 흉내낸다
    ④ /odom 발행             — odom 프레임 기준 (odom_check 등 기존 도구 호환)

  즉 path_planner 와 bridge_node 입장에서는 진짜 로봇과 구분되지 않는다.

[왜 map→odom 을 일부러 틀 수 있게 했나]
  이 리허설의 핵심 검증 항목이다. 실제 SLAM 은 누적 드리프트를 보정하느라
  map→odom 이 0 이 아니다. 예전 코드는 /odom(odom 프레임) 위치와
  /global_path(map 프레임) 를 변환 없이 뺐기 때문에, 이 offset 이 그대로
  추종 오차로 둔갑했다. -p map_odom_x:=0.5 로 틀어놓고도 추종이 되면
  프레임 정합이 제대로 된 것이다.

[이 노드는 젯슨이 아니라 PC 에서 돈다]
  ⚠️ 이 PC 는 conda(파이썬3.13)가 기본 활성화라 rclpy(3.12)가 깨진다.
     반드시 conda 를 끄고 시스템 파이썬으로 돌릴 것. (tools/odom_check.py 와 같은 함정)

[실행]
  conda deactivate                # which python3 가 /usr/bin/python3 가 될 때까지
  ros_setup

  # 터미널 1 — 시뮬레이터
  python3 tools/mpc_sim.py
  #   map→odom 을 틀어서 프레임 정합을 시험하려면:
  #   python3 tools/mpc_sim.py --ros-args -p map_odom_x:=0.5 -p map_odom_y:=0.5

  # 터미널 2 — A* 경로 계획
  ros2 run mpc_tubempc_bridge mpc_tubempc_path_planner

  # 터미널 3 — Tube-MPC
  ros2 run mpc_tubempc_bridge mpc_tubempc_bridge --ros-args \
      -p use_goal_topic:=true -p use_global_path:=true

  # 터미널 4 — 목표 발행
  ros2 topic pub /mpc_goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 2.0}, orientation: {w: 1.0}}}" --once

  # 터미널 5 — 시각화 / 관측
  rviz2 -d src/relayrobot_description/config/nav.rviz
  ros2 topic echo /mpc/status
  rqt_plot /mpc/tracking_error/x /mpc/tracking_error/y /mpc/tracking_error/z

  # 리셋 (로봇을 원점으로)
  ros2 service call /mpc_sim/reset std_srvs/srv/Trigger

[한계 — 이 노드가 확인해주지 못하는 것]
  - 충돌 판정이 없다. 경로가 벽을 뚫으면 로봇도 그냥 통과한다(RViz 로 눈으로 볼 것).
  - 모터 시리얼·watchdog·유령 거리는 여기서 못 본다 → 실기 STEP 1 에서 확인한다.
  - 라이다/IMU 노이즈, 바퀴 미끄러짐 없음. 제어기 로직 검증용이지 성능 예측용이 아니다.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


class MpcSim(Node):

    def __init__(self):
        super().__init__('mpc_sim')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('resolution',  0.05),
                ('size_m',      20.0),   # 정사각 지도 한 변 (m)
                ('map_odom_x',  0.0),    # map→odom offset — 프레임 정합 시험용
                ('map_odom_y',  0.0),
                ('map_odom_yaw', 0.0),
                ('start_x',     0.0),    # map 프레임 기준 초기 위치
                ('start_y',     0.0),
                ('start_yaw',   0.0),
                ('sim_rate',    20.0),
                ('cmd_timeout', 0.5),    # 실기 드라이버 watchdog 과 같은 동작
            ]
        )

        self.res     = float(self.get_parameter('resolution').value)
        self.size_m  = float(self.get_parameter('size_m').value)
        self.ox      = float(self.get_parameter('map_odom_x').value)
        self.oy      = float(self.get_parameter('map_odom_y').value)
        self.oyaw    = float(self.get_parameter('map_odom_yaw').value)
        self.rate    = float(self.get_parameter('sim_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.start = (float(self.get_parameter('start_x').value),
                      float(self.get_parameter('start_y').value),
                      float(self.get_parameter('start_yaw').value))

        # map 프레임 기준 "진짜" 위치. SLAM 이 완벽하다고 가정한 값이다.
        self.x, self.y, self.th = self.start

        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = None

        self.grid, self.info = self.build_map()

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub  = self.create_publisher(OccupancyGrid, 'map',  latched)
        self.odom_pub = self.create_publisher(Odometry,      'odom', 10)
        self.tf_bc    = TransformBroadcaster(self)

        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        self.create_service(Trigger, 'mpc_sim/reset', self.reset_cb)

        self.create_timer(1.0, self.publish_map)
        self.create_timer(1.0 / self.rate, self.step)

        self.publish_map()
        self.get_logger().info(
            f'mpc_sim 시작. 지도 {self.size_m}x{self.size_m}m @ {self.res}m, '
            f'시작 위치 {self.start}, map→odom offset ({self.ox}, {self.oy}, {self.oyaw})')

    # ── 지도 ────────────────────────────────────────────────────────────────

    def build_map(self):
        """테두리 벽 + 중앙 장애물 + 틈 있는 가림벽 + 미탐색 구역."""
        n = int(self.size_m / self.res)
        grid = np.zeros((n, n), dtype=np.int8)   # 0 = 자유

        def m2c(v):   # 월드 좌표(m) → 셀 인덱스. 원점은 지도 중앙.
            return int((v + self.size_m / 2.0) / self.res)

        # 바깥 테두리 벽 (두께 0.1m)
        t = int(0.1 / self.res)
        grid[:t, :] = 100
        grid[-t:, :] = 100
        grid[:, :t] = 100
        grid[:, -t:] = 100

        # 중앙 장애물: (1.5, 0.5) 근처 1.0 x 1.0 m 블록
        grid[m2c(0.0):m2c(1.0), m2c(1.2):m2c(2.2)] = 100

        # 가림벽: x = 4.0 에 세로벽, y = 0.0~0.6 구간만 틈
        wx = m2c(4.0)
        grid[m2c(-3.0):m2c(0.0), wx:wx + t] = 100
        grid[m2c(0.6):m2c(3.0),  wx:wx + t] = 100

        # 미탐색 구역 — allow_unknown=false 일 때 경로가 여기로 안 새는지 확인용
        grid[m2c(3.0):m2c(6.0), m2c(-6.0):m2c(-3.0)] = -1

        info = OccupancyGrid().info
        info.resolution = self.res
        info.width      = n
        info.height     = n
        info.origin.position.x = -self.size_m / 2.0
        info.origin.position.y = -self.size_m / 2.0
        info.origin.orientation.w = 1.0
        return grid, info

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.info = self.info
        msg.data = self.grid.ravel().tolist()
        self.map_pub.publish(msg)

    # ── 명령 / 리셋 ─────────────────────────────────────────────────────────

    def cmd_cb(self, msg: Twist):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def reset_cb(self, request, response):
        self.x, self.y, self.th = self.start
        self.cmd_v = self.cmd_w = 0.0
        self.last_cmd_time = None
        response.success = True
        response.message = f'위치를 {self.start} 로 되돌렸습니다.'
        self.get_logger().info(response.message)
        return response

    # ── 시뮬레이션 한 스텝 ──────────────────────────────────────────────────

    def step(self):
        now = self.get_clock().now()
        dt  = 1.0 / self.rate

        # 실기 드라이버와 같은 watchdog. 명령이 끊기면 선다.
        if self.last_cmd_time is None:
            v = w = 0.0
        elif (now - self.last_cmd_time).nanoseconds / 1e9 > self.cmd_timeout:
            v = w = 0.0
        else:
            v, w = self.cmd_v, self.cmd_w

        # 차동 구동 기구학 (map 프레임 = 진짜 위치)
        self.th = wrap_angle(self.th + w * dt)
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt

        stamp = now.to_msg()
        self.publish_tf(stamp)
        self.publish_odom(stamp, v, w)

    def publish_tf(self, stamp):
        # map → odom : SLAM 이 발행하는 보정분. 여기서는 파라미터로 고정한다.
        t1 = TransformStamped()
        t1.header.stamp    = stamp
        t1.header.frame_id = 'map'
        t1.child_frame_id  = 'odom'
        t1.transform.translation.x = self.ox
        t1.transform.translation.y = self.oy
        t1.transform.rotation = yaw_to_quat(self.oyaw)

        # odom → base_link : EKF 가 발행하는 부분.
        #   T_odom_base = T_map_odom⁻¹ ∘ T_map_base
        # 이렇게 해야 두 TF 를 이으면 진짜 위치(map→base_link)가 정확히 나온다.
        ox, oy, oyaw = self.publish_odom_pose()
        t2 = TransformStamped()
        t2.header.stamp    = stamp
        t2.header.frame_id = 'odom'
        t2.child_frame_id  = 'base_link'
        t2.transform.translation.x = ox
        t2.transform.translation.y = oy
        t2.transform.rotation = yaw_to_quat(oyaw)

        self.tf_bc.sendTransform([t1, t2])

    def publish_odom_pose(self):
        """map 프레임 진짜 위치를 odom 프레임 좌표로 환산."""
        dx, dy = self.x - self.ox, self.y - self.oy
        c, s = math.cos(self.oyaw), math.sin(self.oyaw)
        return (c * dx + s * dy,
                -s * dx + c * dy,
                wrap_angle(self.th - self.oyaw))

    def publish_odom(self, stamp, v, w):
        ox, oy, oyaw = self.publish_odom_pose()
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = ox
        odom.pose.pose.position.y = oy
        odom.pose.pose.orientation = yaw_to_quat(oyaw)
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = MpcSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
