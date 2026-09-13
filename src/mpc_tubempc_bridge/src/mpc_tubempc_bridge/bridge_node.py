import sys
from pathlib import Path as FsPath
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# TubeMPCPlanner는 ddsm_example/mpc_tubempc/ 에 있음
# 별도 패키지가 아니라 sys.path로 직접 가져옴 → colcon build --symlink-install 필수
this_dir = FsPath(__file__).resolve().parent
mpc_dir  = this_dir.parents[2] / 'ddsm_example' / 'mpc_tubempc'
if str(mpc_dir) not in sys.path:
    sys.path.insert(0, str(mpc_dir))

from TubeMPCPlanner import TubeMPCPlanner


def wrap_angle(angle: float) -> float:
    """각도를 -π ~ +π 범위로 정규화."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)


class MPCBridgeNode(Node):
    """
    역할: Tube-MPC 제어기 — 현재 위치(odom)와 목표 경로(global_path)를 받아
          최적 속도 명령(cmd_vel)을 계산해서 모터 드라이버로 전달

    구독 토픽:
      /mpc_goal     → 목표 좌표 (global_frame 기준 x, y, 방향)
      /global_path  → A* 경로 계획기가 계산한 경유점 리스트
    발행 토픽:
      /cmd_vel              → 모터 드라이버로 전달되는 선속도(v) + 각속도(ω)
      /mpc/reference_path   → 지금 호라이즌의 참조 궤적 (원격 PC RViz 확인용)
      /mpc/tracking_error   → 추종 오차 e_act (x, y, θ) — rqt_plot 용
      /mpc/status           → RUNNING / QP_FAILED / GOAL_REACHED / NO_POSE

    현재 위치는 토픽이 아니라 TF(global_frame → base_frame)로 받는다.
    /odom 은 odom 프레임인데 경로·목표는 map 프레임이라, 토픽을 그대로 쓰면
    SLAM 이 드리프트를 보정한 만큼이 그대로 추종 오차로 둔갑한다.

    이 노드는 로봇(젯슨)에서 돈다. 10Hz 제어 루프를 무선 너머에 두면
    통신 끊김이 곧 제어 지터가 된다. 원격 PC 는 위 /mpc/* 토픽으로 관찰만 한다.

    파라미터 (실행 시 --ros-args -p 로 조정 가능):
      global_frame:   경로·목표·위치의 기준 프레임. 기본 map.
                      SLAM 없이 시험할 때는 odom 으로 내린다.
      velocity_limit: 최대 선속도 (m/s), 기본 0.2
      omega_limit:    최대 각속도 (rad/s), 기본 1.0
      horizon:        MPC 예측 스텝 수, 기본 4 (augmented model이 4스텝 고정이라 4 권장)
    """

    def __init__(self):
        super().__init__('mpc_tubempc_bridge')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('global_frame',    'map'),
                ('base_frame',       'base_link'),
                ('goal_x',          0.0),
                ('goal_y',          0.0),
                ('goal_theta',      0.0),
                ('use_goal_topic',  False),   # True: /mpc_goal 토픽으로 목표 수신
                ('use_global_path', False),   # True: A* 경로 추종, False: 직선 목표 추종
                ('velocity_limit',  0.2),
                ('omega_limit',     1.0),
                ('publish_rate',    10.0),
                ('horizon',         4),
                ('simulation_time', 20.0),
                ('goal_tolerance',  0.1),    # 목표 반경 (m) 안으로 들어오면 정지
                ('error_xy_limit',  2.0),    # 허용 위치 오차 (m)
                ('error_yaw_limit', 3.2),    # 허용 heading 오차 (rad) — 아래 주석 참고
                ('path_timeout',    2.0),    # 이보다 오래된 /global_path 는 없는 것으로 본다
            ]
        )

        self.goal_pose = np.array([
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value,
            self.get_parameter('goal_theta').value,
        ], dtype=float)
        self.global_frame    = self.get_parameter('global_frame').value
        self.base_frame      = self.get_parameter('base_frame').value
        self.use_goal_topic  = self.get_parameter('use_goal_topic').value
        self.use_global_path = self.get_parameter('use_global_path').value
        self.max_v           = self.get_parameter('velocity_limit').value
        self.max_w           = self.get_parameter('omega_limit').value
        self.publish_rate    = self.get_parameter('publish_rate').value
        self.horizon         = int(self.get_parameter('horizon').value)
        self.simulation_time = float(self.get_parameter('simulation_time').value)
        self.goal_tolerance  = float(self.get_parameter('goal_tolerance').value)
        self.error_xy_limit  = float(self.get_parameter('error_xy_limit').value)
        self.error_yaw_limit = float(self.get_parameter('error_yaw_limit').value)
        self.path_timeout    = float(self.get_parameter('path_timeout').value)

        self.global_path           = None
        self.global_path_time      = None   # 마지막 /global_path 수신 시각
        # tube 명목 오차 상태: 사이클 간 전파됨. 목표/경로가 갱신되면 None으로 리셋해
        # 다음 사이클에서 실측 오차로 재정렬한다.
        self.e_nom                 = None

        # 현재 위치는 TF 로 받는다 (프레임 정합 — 위 docstring 참고)
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if self.use_goal_topic:
            self.goal_sub = self.create_subscription(
                PoseStamped, 'mpc_goal', self.goal_callback, 10,
            )

        if self.use_global_path:
            self.path_sub = self.create_subscription(
                Path, 'global_path', self.global_path_callback, 10,
            )

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 원격 관측용. 이게 없으면 MPC 상태가 젯슨 터미널 로그 안에만 갇힌다.
        self.ref_path_pub = self.create_publisher(Path,    'mpc/reference_path', 10)
        self.error_pub    = self.create_publisher(Vector3, 'mpc/tracking_error', 10)
        self.status_pub   = self.create_publisher(String,  'mpc/status',         10)

        self.timer   = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.mpc = self._create_mpc_planner()
        self.get_logger().info(
            f'MPC Tube bridge initialized. frame={self.global_frame}, '
            f'v_max={self.max_v}, w_max={self.max_w}, horizon={self.horizon}')

    def lookup_pose(self):
        """global_frame 기준 [x, y, yaw]. 아직 TF가 없으면 None."""
        try:
            # timeout 을 주면 안 된다. rclpy 의 spin 은 단일 스레드라 이 함수가 자는 동안
            # /tf 메시지가 아예 배달되지 않는다 — 기다려도 절대 성공하지 않고 제어 주기만 잡아먹는다.
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, Time())
        except TransformException as e:
            self.get_logger().warn(
                f'TF {self.global_frame}→{self.base_frame} 없음: {e}',
                throttle_duration_sec=5.0)
            return None
        return np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            self._quaternion_to_yaw(tf.transform.rotation),
        ], dtype=float)

    def publish_status(self, state: str):
        msg = String()
        msg.data = state
        self.status_pub.publish(msg)

    def publish_reference_path(self, qRef):
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp    = self.get_clock().now().to_msg()
        for i in range(qRef.shape[1]):
            pose        = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(qRef[0, i])
            pose.pose.position.y = float(qRef[1, i])
            pose.pose.orientation.z = math.sin(qRef[2, i] / 2.0)
            pose.pose.orientation.w = math.cos(qRef[2, i] / 2.0)
            path.poses.append(pose)
        self.ref_path_pub.publish(path)

    def goal_callback(self, msg: PoseStamped):
        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            self.get_logger().warn(
                f"목표 프레임이 '{msg.header.frame_id}' 인데 이 노드 기준은 "
                f"'{self.global_frame}' 입니다. 변환하지 않고 그대로 씁니다 — 좌표를 확인하세요.")
        self.goal_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            self._quaternion_to_yaw(msg.pose.orientation),
        ], dtype=float)
        self.e_nom = None   # 새 목표 → 명목 상태 재정렬
        self.get_logger().info(f'Goal updated: {self.goal_pose}')

    def global_path_callback(self, msg: Path):
        # 계획기는 실패했을 때 "빈 경로" 를 보낸다 = 갈 길이 없다는 뜻.
        # 이걸 무시하고 옛 경로를 계속 들고 있으면, 지도에 새 장애물이 생겨
        # 경로가 막혔는데도 낡은 경로를 그대로 따라간다.
        if not msg.poses:
            self.global_path      = None
            self.global_path_time = None
            self.e_nom            = None
            self.get_logger().warn('빈 /global_path 수신 — 경로 없음으로 처리',
                                   throttle_duration_sec=5.0)
            return
        self.global_path_time = self.get_clock().now()
        self.global_path = [
            np.array([
                pose.pose.position.x,
                pose.pose.position.y,
                self._quaternion_to_yaw(pose.pose.orientation),
            ], dtype=float)
            for pose in msg.poses
        ]
        self.e_nom = None   # 새 경로 → 명목 상태 재정렬
        self.get_logger().info(f'Received global path with {len(self.global_path)} points.')

    def timer_callback(self):
        current = self.lookup_pose()
        if current is None:
            self.publish_status('NO_POSE')
            self.cmd_pub.publish(Twist())   # 위치를 모르면 달리지 않는다
            return

        # 경로가 아직/이미 유효하지 않으면 달리지 않는다.
        #
        # ★ 왜 직선 폴백을 쓰지 않나: 예전 코드는 경로가 없으면 목표까지 직선 참조를 만들어
        #   그대로 달렸다. 그런데 A* 실패("경로 없음"·"목표가 막힘")나 계획기 사망도 똑같이
        #   "경로 없음" 이다. 즉 지도를 전혀 안 보고 장애물을 향해 직진하게 된다.
        #   경로 추종 모드에서 경로가 없으면 **서는 것**이 맞다.
        path_ok = True
        if self.use_global_path:
            if self.global_path is None or len(self.global_path) < 2:
                path_ok = False
            else:
                age = (self.get_clock().now() - self.global_path_time).nanoseconds / 1e9
                if age > self.path_timeout:
                    # 계획기가 죽었거나 계획을 못 내고 있다 → 낡은 경로를 따라가면 안 된다
                    path_ok = False
                    self.get_logger().warn(
                        f'/global_path 가 {age:.1f}s 낡음 (>{self.path_timeout}s) -> 정지',
                        throttle_duration_sec=5.0)
        if not path_ok:
            self.cmd_pub.publish(Twist())
            self.publish_status('NO_PATH')
            self.e_nom = None
            return

        # 최종 목표 도달 판정 → 정지
        if self.use_global_path:
            final_goal = self.global_path[-1]
        else:
            final_goal = self.goal_pose
        if np.hypot(final_goal[0] - current[0], final_goal[1] - current[1]) < self.goal_tolerance:
            self.cmd_pub.publish(Twist())   # zero stop
            self.publish_status('GOAL_REACHED')
            return

        # 참조 궤적(world 프레임) + 참조 입력(uRef: 피드포워드) 생성
        if self.use_global_path:
            qRef, uRef = self._generate_reference_from_path(current, self.global_path)
        else:
            qRef, uRef = self._generate_reference(current, self.goal_pose.copy())

        # 실측 추종오차(로봇 프레임): 참조점 qRef[:,0] 대비 현재 pose
        e_act = self.mpc.compute_error(current, qRef[:, 0])
        self.publish_reference_path(qRef)
        self.error_pub.publish(Vector3(x=float(e_act[0]),
                                       y=float(e_act[1]),
                                       z=float(e_act[2])))

        # tube 명목 상태 초기화(목표/경로 갱신 직후 실측으로 재정렬)
        if self.e_nom is None:
            self.e_nom = e_act.copy()

        # 선형화 행렬: 각 스텝의 참조 속도(v, ω) 기준
        A0 = self._A_matrix(uRef[0, 0], uRef[1, 0])
        A1 = self._A_matrix(uRef[0, 1], uRef[1, 1]) if self.horizon > 1 else A0
        A2 = self._A_matrix(uRef[0, 2], uRef[1, 2]) if self.horizon > 2 else A0
        A3 = self._A_matrix(uRef[0, 3], uRef[1, 3]) if self.horizon > 3 else A0

        # 명목 시스템(e_nom) 기준으로 QP 풀이. 제약은 __init__에서 이미 tube만큼
        # 타이트닝돼 있어 ancillary 보정 여유를 남겨둔다.
        B_bar, A_bar = self.mpc.construct_augmentemd_model(A0, A1, A2, A3)
        Uad_A, Uad_b = self.mpc.construct_constraint_matrices(B_bar, A0, A1, A2, A3, self.e_nom)
        H_qp,  f_qp  = self.mpc.construct_cost_matrices(B_bar, A_bar, self.e_nom)

        try:
            # u_nom: 참조 입력(uRef) 대비 명목 보정량 (Δv, Δω)
            u_nom = self.mpc.solve_qp(H_qp, f_qp, Uad_A, Uad_b)
        except Exception as e:
            self.get_logger().warn(f'MPC QP failed: {e}', throttle_duration_sec=2.0)
            self.cmd_pub.publish(Twist())
            self.publish_status('QP_FAILED')
            return

        # Tube ancillary: 실측-명목 편차를 K로 되먹임해 tube 안에 가둠
        u_corr = u_nom - self.mpc.K @ (e_act - self.e_nom)

        # 최종 명령 = 피드포워드(참조 입력) + 보정량, 속도 한계로 클램프
        v_cmd = clamp(uRef[0, 0] + u_corr[0], -self.max_v, self.max_v)
        w_cmd = clamp(uRef[1, 0] + u_corr[1], -self.max_w, self.max_w)

        # 명목 상태 1스텝 전파 → 다음 사이클로 이어짐 (tube 핵심).
        #
        # ★ u_nom 이 아니라 "실제로 낸 보정량" 으로 전파한다. QP 의 입력집합(±0.5)은
        #   속도 한계(velocity_limit, 예 0.1)보다 넓어서 위 clamp 가 자주 걸린다.
        #   그때 u_nom 으로 전파하면 명목 상태가 현실과 계속 벌어지고,
        #   ancillary 항 K(e_act - e_nom) 이 허구를 기준으로 커진다.
        u_applied = np.array([v_cmd - uRef[0, 0], w_cmd - uRef[1, 0]], dtype=float)
        self.e_nom = A0 @ self.e_nom + self.mpc.B @ u_applied

        cmd           = Twist()
        cmd.linear.x  = float(v_cmd)
        cmd.angular.z = float(w_cmd)
        self.cmd_pub.publish(cmd)

        self.publish_status('RUNNING')
        # 10Hz 로 info 를 찍으면 tmux 로그가 폭주한다. 원격에서는 /mpc/* 토픽으로 본다.
        self.get_logger().debug(
            f'pose={current.round(2)}, e_act={e_act.round(3)}, cmd=[{v_cmd:.3f}, {w_cmd:.3f}]')

    def _create_mpc_planner(self):
        """TubeMPCPlanner 초기화: 상태/입력 제약 집합, LQR gain, B행렬 설정."""
        Ts    = 0.1
        # 허용 오차 범위 (x_err, y_err, θ_err).
        #
        # θ 한계가 왜 ±3.2 (사실상 무제한) 인가 — 2026-09-13 측정으로 확정:
        #   예전 값 -0.3 은 tube 만큼 타이트닝되면 실효 0.25 rad(14°)가 된다.
        #   그런데 A* 경로를 따라갈 때 첫 참조점의 접선 방향과 로봇의 현재 방향은
        #   쉽게 20~30° 벌어진다(출발 시 경로가 옆으로 나가면 항상 그렇다).
        #   그러면 첫 사이클부터 상태 제약이 이미 위반이라 QP 가 100% infeasible 이 되고,
        #   로봇은 제자리에서 멈춘다. 실측: 실패 경계가 정확히 0.25 rad 였다.
        #
        #   heading 오차는 안전 제약이 아니다 — 돌아서 줄이면 되는 값이다.
        #   실제로 지켜야 할 것은 위치 오차(tube)이고 그건 x/y 한계와 ancillary
        #   되먹임이 담당한다. 그래서 θ 는 풀어주고 위치만 묶는다.
        x_min = np.array([-self.error_xy_limit,
                          -self.error_xy_limit,
                          -self.error_yaw_limit])
        u_min = np.array([-0.5, -0.5])           # 최소 제어입력 (v, ω)
        w_min = np.array([-0.1, -0.1, -0.1])     # 허용 외란 범위
        e_min = np.array([-0.05, -0.05, -0.05])  # tube 크기 (작게: 타이트닝된 입력집합이
                                                 #            공집합이 되지 않도록)

        v0 = 0.05
        w0 = 0.0
        A  = self._A_matrix(v0, w0)
        B  = np.array([[Ts, 0.0], [0.0, 0.0], [0.0, Ts]])
        Q  = 100 * np.eye(3)   # 상태 오차 가중치 (클수록 경로 추종 우선)
        R  = 5.0 * np.eye(2)   # 입력 가중치. 작으면 LQR gain K가 과격(≈10)해져
                               # tube 타이트닝량(K·e)이 입력 한계를 넘어 QP가 항상
                               # infeasible해진다 → 5.0으로 K를 완화해 feasibility 확보
        P  = self._solve_are(A, B, Q, R)
        K  = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)

        return TubeMPCPlanner(
            Ts, self.simulation_time, self.horizon,
            x_min, u_min, w_min, e_min, K, B,
            self._A_matrix_from_ref,
        )

    def _A_matrix(self, v, w):
        """차동 구동 로봇의 오차 동역학 선형화 행렬 (이산 시간, Ts=0.1s)."""
        Ts = 0.1
        return np.array([
            [1.0,   Ts * w, 0.0   ],
            [-Ts*w, 1.0,    Ts * v],
            [0.0,   0.0,    1.0   ],
        ])

    def _A_matrix_from_ref(self, k, offset):
        v = self.ref_u[0, min(k + offset, self.horizon - 1)]
        w = self.ref_u[1, min(k + offset, self.horizon - 1)]
        return self._A_matrix(v, w)

    def _generate_reference(self, current, goal):
        """목표점까지 직선 참조 궤적 생성 (A* 경로 없을 때)."""
        delta          = goal - current
        dist           = float(np.hypot(delta[0], delta[1]))
        heading_to_goal = math.atan2(delta[1], delta[0])
        heading_error  = wrap_angle(heading_to_goal - current[2])

        v_nom = clamp(dist / (self.horizon * 0.1), 0.0, self.max_v)
        w_nom = clamp(heading_error / (self.horizon * 0.1), -self.max_w, self.max_w)

        qRef       = np.zeros((3, self.horizon + 1))
        uRef       = np.zeros((2, self.horizon))
        qRef[:, 0] = current
        self.ref_u = np.zeros((2, self.horizon))

        for i in range(self.horizon):
            self.ref_u[:, i]   = [v_nom, w_nom]
            q                  = qRef[:, i]
            qRef[0, i + 1]     = q[0] + v_nom * 0.1 * math.cos(q[2])
            qRef[1, i + 1]     = q[1] + v_nom * 0.1 * math.sin(q[2])
            qRef[2, i + 1]     = wrap_angle(q[2] + w_nom * 0.1)

        uRef[:] = self.ref_u
        return qRef, uRef

    def _generate_reference_from_path(self, current, path):
        """world 프레임 A* 경로에서 추종 참조 궤적 생성.

        qRef[:,0] 을 현재 위치 최근접 경로점으로 잡아 실제 추종오차(cross-track)가
        생기게 하고, 이후 호라이즌은 참조 속도(max_v)만큼 경로를 따라 전진시킨다.
        uRef 는 경로 접선 방향으로의 참조 속도(v, ω) — 피드포워드로 쓰인다.
        """
        pts = np.array([[p[0], p[1]] for p in path], dtype=float)

        # 현재 위치에서 가장 가까운 경로점 = "지금 있어야 할 위치"
        closest = int(np.argmin(np.hypot(pts[:, 0] - current[0], pts[:, 1] - current[1])))
        step_len = self.max_v * 0.1   # Ts=0.1s 동안 전진할 거리

        qRef = np.zeros((3, self.horizon + 1))
        uRef = np.zeros((2, self.horizon))

        # 경로를 따라 정확히 step_len 씩 전진한 점을 만든다.
        #
        # ★ 예전에는 셀 인덱스 단위로 건너뛰었다. A* 한 셀은 0.05m(대각 0.07m)인데
        #   step_len 은 max_v*0.1 = 0.01~0.02m 라, 한 스텝에 최소 한 셀을 통째로 넘어가
        #   참조 궤적이 실제 속도 한계의 5~7배로 달아났다. 그 결과
        #   (a) 예측 모델과 참조가 서로 안 맞고
        #   (b) uRef[0] 이 항상 max_v 에 붙어, 경로가 옆으로 꺾여도 전속 전진 명령이 나갔다.
        #   세그먼트 안에서 보간해 실제로 갈 수 있는 궤적을 만든다.
        s_along    = self._path_arclength(pts, closest)
        qRef[:, 0] = self._path_pose_at(pts, s_along)
        for i in range(self.horizon):
            s_along       += step_len
            qRef[:, i + 1] = self._path_pose_at(pts, s_along)
            dx = qRef[0, i + 1] - qRef[0, i]
            dy = qRef[1, i + 1] - qRef[1, i]
            v_ref = float(np.hypot(dx, dy)) / 0.1
            w_ref = wrap_angle(qRef[2, i + 1] - qRef[2, i]) / 0.1
            uRef[:, i] = [clamp(v_ref, 0.0, self.max_v),
                          clamp(w_ref, -self.max_w, self.max_w)]

        self.ref_u = uRef.copy()
        return qRef, uRef

    def _path_arclength(self, pts, idx):
        """경로 시작점부터 idx 번째 점까지의 누적 거리(m)."""
        if idx <= 0:
            return 0.0
        seg = np.hypot(np.diff(pts[:idx + 1, 0]), np.diff(pts[:idx + 1, 1]))
        return float(seg.sum())

    def _path_pose_at(self, pts, s):
        """경로 시작점에서 s(m) 만큼 간 지점의 [x, y, heading].

        셀에 스냅하지 않고 세그먼트 안에서 선형 보간한다. s 가 경로 길이를 넘으면
        마지막 점에서 멈춘다(경로 끝에 붙어 감속하게 된다).
        """
        if len(pts) < 2:
            return np.array([pts[0, 0], pts[0, 1], 0.0], dtype=float)

        seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
        cum = np.concatenate(([0.0], np.cumsum(seg)))
        s   = float(np.clip(s, 0.0, cum[-1]))

        j = int(np.searchsorted(cum, s, side='right') - 1)
        j = max(0, min(j, len(seg) - 1))
        t = 0.0 if seg[j] <= 1e-9 else (s - cum[j]) / seg[j]

        x = pts[j, 0] + t * (pts[j + 1, 0] - pts[j, 0])
        y = pts[j, 1] + t * (pts[j + 1, 1] - pts[j, 1])
        heading = math.atan2(pts[j + 1, 1] - pts[j, 1], pts[j + 1, 0] - pts[j, 0])
        return np.array([x, y, heading], dtype=float)

    def _solve_are(self, A, B, Q, R):
        from scipy.linalg import solve_discrete_are
        return solve_discrete_are(A, B, Q, R)

    def _quaternion_to_yaw(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )


def main(args=None):
    rclpy.init(args=args)
    node = MPCBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
