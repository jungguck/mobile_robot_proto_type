import heapq
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.ndimage import distance_transform_edt
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class AStarPlanner(Node):
    """
    역할: Cartographer가 만든 지도(/map) + 현재 위치(TF) + 목표(/mpc_goal)를
          받아서 A* 알고리즘으로 경로 계산 → /global_path 발행

    구독 토픽:
      /map      → Cartographer SLAM이 생성한 격자 지도 (OccupancyGrid)
      /mpc_goal → 목표 좌표 (bridge_node와 동일 토픽 공유, global_frame 기준)
    발행 토픽:
      /global_path  → 계산된 경유점 리스트 → bridge_node가 MPC 참조 궤적으로 사용
      /inflated_map → 로봇 반경만큼 부풀린 지도. 원격 PC RViz 에서 "왜 경로가
                      안 나오는지" 를 눈으로 확인하는 용도.

    현재 위치는 토픽이 아니라 TF(global_frame → base_frame)로 받는다.
    /odom 은 odom 프레임이고 지도·목표는 map 프레임이라, 토픽을 그대로 쓰면
    SLAM 이 누적 드리프트를 보정한 만큼이 그대로 위치 오차가 된다.

    파라미터:
      global_frame:     지도/목표/경로의 기준 프레임. 기본 map.
                        SLAM 없이 시험할 때는 odom 으로 내린다.
      robot_radius:     로봇 외접 반경 (m). 이만큼 장애물을 부풀린다.
      inflation_margin: 추가 여유 (m).
      allow_unknown:    미탐색(-1) 셀 통행 허용 여부. 기본 False.
    """

    def __init__(self):
        super().__init__('mpc_tubempc_path_planner')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('global_frame',     'map'),
                ('base_frame',       'base_link'),
                ('robot_radius',     0.20),   # 바퀴 중심간 0.22m → 외접 반경 ≈ 0.2m
                ('inflation_margin', 0.05),
                ('allow_unknown',    False),
                ('plan_period',      1.0),
                ('start_search_radius', 20),  # 시작 셀 대체 탐색 반경 (셀)
                ('max_expansions', 120000),   # A* 노드 전개 상한 (무한 탐색 방지)
            ]
        )

        self.global_frame     = self.get_parameter('global_frame').value
        self.base_frame       = self.get_parameter('base_frame').value
        self.robot_radius     = float(self.get_parameter('robot_radius').value)
        self.inflation_margin = float(self.get_parameter('inflation_margin').value)
        self.allow_unknown    = bool(self.get_parameter('allow_unknown').value)
        self.start_search_radius = int(self.get_parameter('start_search_radius').value)
        self.max_expansions      = int(self.get_parameter('max_expansions').value)
        plan_period           = float(self.get_parameter('plan_period').value)

        self.map_info  = None
        self.free_mask = None   # True = 로봇이 들어가도 되는 셀 (팽창 반영 완료)
        self.blocked   = None   # True = 원본 장애물 (팽창 전). 가시선 검사용.
        self.goal      = None

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, 'map',      self.map_callback,  10)
        self.create_subscription(PoseStamped,   'mpc_goal', self.goal_callback, 10)

        self.path_pub = self.create_publisher(Path, 'global_path', 10)

        # RViz 의 Map 디스플레이는 transient_local 로 붙는다. 늦게 띄운 RViz 도
        # 마지막 지도를 받도록 publisher 쪽을 transient_local 로 둔다.
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.inflated_pub = self.create_publisher(OccupancyGrid, 'inflated_map', latched)

        self.timer = self.create_timer(plan_period, self.plan_callback)
        self.get_logger().info(
            f'A* Path Planner initialized. frame={self.global_frame}, '
            f'robot_radius={self.robot_radius}m (+{self.inflation_margin}m), '
            f'allow_unknown={self.allow_unknown}')

    # ── 지도 ────────────────────────────────────────────────────────────────

    def map_callback(self, msg: OccupancyGrid):
        """지도가 올 때마다 팽창을 한 번만 계산해 둔다(계획마다 다시 하지 않는다)."""
        grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        res  = msg.info.resolution

        # OccupancyGrid: 0=빈공간, 100=장애물, -1=미탐색
        blocked = (grid >= 50)

        # 각 자유 셀에서 "가장 가까운 장애물까지의 거리". 셀 단위 판정만 하면
        # 로봇 몸통이 벽을 긁는 경로가 나온다 — 반경만큼 떨어진 셀만 허용한다.
        if blocked.any():
            dist = distance_transform_edt(~blocked) * res
        else:
            dist = np.full(grid.shape, np.inf)

        free = dist > (self.robot_radius + self.inflation_margin)

        # 미탐색을 통행 가능으로 두면 경로가 지도 밖으로 뚫고 나간다.
        if not self.allow_unknown:
            free &= (grid != -1)

        self.free_mask = free
        self.blocked   = blocked
        self.map_info  = msg.info
        self.publish_inflated_map(msg, free)

    def publish_inflated_map(self, src: OccupancyGrid, free: np.ndarray):
        out = OccupancyGrid()
        out.header = src.header
        out.header.frame_id = src.header.frame_id or self.global_frame
        out.info   = src.info
        out.data   = np.where(free, 0, 100).astype(np.int8).ravel().tolist()
        self.inflated_pub.publish(out)

    def publish_empty_path(self):
        """계획 실패를 '빈 경로' 로 알린다.

        아무것도 안 보내면 MPC 는 직전 경로를 계속 유효한 것으로 알고 따라간다.
        지도에 새 장애물이 생겨 길이 막혔을 때 이게 사고가 된다.
        """
        msg = Path()
        msg.header.frame_id = self.global_frame
        msg.header.stamp    = self.get_clock().now().to_msg()
        self.path_pub.publish(msg)

    def goal_callback(self, msg: PoseStamped):
        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            self.get_logger().warn(
                f"목표 프레임이 '{msg.header.frame_id}' 인데 이 노드 기준은 "
                f"'{self.global_frame}' 입니다. 변환하지 않고 그대로 씁니다 — 좌표를 확인하세요.")
        self.goal = msg.pose
        self.get_logger().info(
            f'Goal received: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')

    # ── 현재 위치 (TF) ──────────────────────────────────────────────────────

    def lookup_pose(self):
        """global_frame 기준 현재 위치 (x, y). 아직 TF가 없으면 None."""
        try:
            # timeout 을 주면 안 된다. rclpy 의 spin 은 단일 스레드라 이 함수가 자는 동안
            # /tf 메시지가 배달되지 않는다 — 기다려도 성공하지 않고 계획 주기만 잡아먹는다.
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, Time())
        except TransformException as e:
            self.get_logger().warn(
                f'TF {self.global_frame}→{self.base_frame} 없음: {e}',
                throttle_duration_sec=5.0)
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    # ── 경로 계획 ───────────────────────────────────────────────────────────

    def plan_callback(self):
        if self.free_mask is None or self.goal is None:
            return

        pose = self.lookup_pose()
        if pose is None:
            return

        start = self.world_to_map(pose[0], pose[1])
        goal  = self.world_to_map(self.goal.position.x, self.goal.position.y)

        # 로봇이 벽 가까이 있으면 팽창 때문에 시작 셀이 막힌 것으로 나온다.
        # 이때 계획 자체를 포기하면 벽 근처에서 영영 못 움직인다 → 가까운 자유 셀로 대체.
        if not self.is_free(start):
            relocated = self.nearest_free(start)
            if relocated is None:
                self.get_logger().warn(
                    'Start cell is blocked and no free cell nearby.',
                    throttle_duration_sec=5.0)
                self.publish_empty_path()
                return
            self.get_logger().warn(
                f'Start cell blocked → 가까운 자유 셀 {relocated} 로 대체',
                throttle_duration_sec=5.0)
            start = relocated

        if not self.is_free(goal):
            self.get_logger().warn('Goal cell is not free.', throttle_duration_sec=5.0)
            self.publish_empty_path()
            return

        t0   = self.get_clock().now()
        path = self.a_star(start, goal)
        elapsed = (self.get_clock().now() - t0).nanoseconds / 1e9

        if path is None:
            self.get_logger().warn('Path planning failed.', throttle_duration_sec=5.0)
            self.publish_empty_path()
            return

        path_msg = Path()
        path_msg.header.frame_id = self.global_frame
        path_msg.header.stamp    = self.get_clock().now().to_msg()

        for mx, my in path:
            pose_msg        = PoseStamped()
            pose_msg.header = path_msg.header
            pose_msg.pose.position.x, pose_msg.pose.position.y = self.map_to_world(mx, my)
            pose_msg.pose.position.z    = 0.0
            pose_msg.pose.orientation.w = 1.0
            path_msg.poses.append(pose_msg)

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f'Published global path ({len(path)} points, {elapsed * 1000:.0f} ms).')

    # ── 격자 ↔ 월드 ─────────────────────────────────────────────────────────

    def world_to_map(self, x: float, y: float):
        """월드 좌표(m) → 격자 인덱스(픽셀)."""
        origin = self.map_info.origin.position
        res    = self.map_info.resolution
        mx     = int((x - origin.x) / res)
        my     = int((y - origin.y) / res)
        return mx, my

    def map_to_world(self, mx: int, my: int):
        """격자 인덱스 → 월드 좌표 중심점."""
        origin = self.map_info.origin.position
        res    = self.map_info.resolution
        x      = origin.x + (mx + 0.5) * res
        y      = origin.y + (my + 0.5) * res
        return x, y

    def in_bounds(self, cell):
        x, y = cell
        return 0 <= x < self.map_info.width and 0 <= y < self.map_info.height

    def is_free(self, cell):
        """팽창까지 반영해 로봇이 들어가도 되는 셀인지."""
        if not self.in_bounds(cell):
            return False
        return bool(self.free_mask[cell[1], cell[0]])

    def nearest_free(self, cell):
        """cell 주변에서 가장 가까운 자유 셀. 없으면 None.

        ★ 거리만 보면 안 된다. 벽 두께가 팽창 지름(2×(robot_radius+margin)=0.5m)보다
        얇으면 **벽 건너편 셀이 더 가깝게 잡힌다.** 그러면 로봇이 물리적으로 갈 수 없는
        곳에서 출발하는 경로가 나오고, MPC 는 그 경로로 붙으려다 벽으로 돌진한다.
        그래서 원본(팽창 전) 장애물 기준으로 **가시선이 뚫려 있는 셀만** 고른다.
        """
        for r in range(1, self.start_search_radius + 1):
            best, best_d = None, None
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    # 이미 본 안쪽은 건너뛰고 테두리만
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    cand = (cell[0] + dx, cell[1] + dy)
                    if not self.is_free(cand):
                        continue
                    if not self.line_of_sight(cell, cand):
                        continue
                    d = dx * dx + dy * dy
                    if best_d is None or d < best_d:
                        best, best_d = cand, d
            if best is not None:
                return best
        return None

    def line_of_sight(self, a, b):
        """a→b 직선이 원본 장애물을 통과하지 않는지 (반 셀 간격으로 표본 검사)."""
        steps = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
        for i in range(steps + 1):
            t = i / steps
            x = int(round(a[0] + t * (b[0] - a[0])))
            y = int(round(a[1] + t * (b[1] - a[1])))
            if not self.in_bounds((x, y)) or self.blocked[y, x]:
                return False
        return True

    # ── A* ──────────────────────────────────────────────────────────────────

    def neighbors(self, cell):
        """8방향 이웃 셀과 이동 비용 반환 (대각선은 √2 비용)."""
        x, y = cell
        for dx, dy, cost in [
            (-1,  0, 1.0), ( 1,  0, 1.0), ( 0, -1, 1.0), ( 0,  1, 1.0),
            (-1, -1, math.sqrt(2)), (-1,  1, math.sqrt(2)),
            ( 1, -1, math.sqrt(2)), ( 1,  1, math.sqrt(2)),
        ]:
            neighbor = (x + dx, y + dy)
            if self.is_free(neighbor):
                yield neighbor, cost

    def heuristic(self, a, b):
        """유클리드 거리 휴리스틱."""
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def a_star(self, start, goal):
        """A* 탐색. 경로 없으면 None 반환."""
        open_set  = [(0.0, start)]
        came_from = {}
        g_score   = {start: 0.0}
        closed    = set()

        while open_set:
            # 도달 불가능한 목표를 주면 자유 공간 전체를 매 주기 훑는다. 순수 파이썬이고
            # 단일 스레드라 그동안 map_callback 과 TF 수신이 통째로 멈춘다 → 상한을 둔다.
            if len(closed) > self.max_expansions:
                self.get_logger().warn(
                    f'A* 전개 상한({self.max_expansions}) 초과 — 중단. '
                    f'목표가 도달 불가능하거나 지도가 너무 큽니다.',
                    throttle_duration_sec=5.0)
                return None

            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)
            if current in closed:
                continue
            closed.add(current)

            for neighbor, cost in self.neighbors(current):
                tentative_g = g_score[current] + cost
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor]   = tentative_g
                    f_score             = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return list(reversed(path))


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
