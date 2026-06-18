import sys
from pathlib import Path
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

# TubeMPCPlanner는 ddsm_example/mpc_tubempc/ 에 있음
# 별도 패키지가 아니라 sys.path로 직접 가져옴 → colcon build --symlink-install 필수
this_dir = Path(__file__).resolve().parent
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
      /odom         → 현재 로봇 위치 (EKF 융합 결과)
      /mpc_goal     → 목표 좌표 (map 기준 x, y, 방향)
      /global_path  → A* 경로 계획기가 계산한 경유점 리스트
    발행 토픽:
      /cmd_vel      → 모터 드라이버로 전달되는 선속도(v) + 각속도(ω)

    파라미터 (실행 시 --ros-args -p 로 조정 가능):
      velocity_limit: 최대 선속도 (m/s), 기본 0.2
      omega_limit:    최대 각속도 (rad/s), 기본 1.0
      horizon:        MPC 예측 스텝 수, 기본 4 (augmented model이 4스텝 고정이라 4 권장)
    """

    def __init__(self):
        super().__init__('mpc_tubempc_bridge')

        self.declare_parameters(
            namespace='',
            parameters=[
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
            ]
        )

        self.goal_pose = np.array([
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value,
            self.get_parameter('goal_theta').value,
        ], dtype=float)
        self.use_goal_topic  = self.get_parameter('use_goal_topic').value
        self.use_global_path = self.get_parameter('use_global_path').value
        self.max_v           = self.get_parameter('velocity_limit').value
        self.max_w           = self.get_parameter('omega_limit').value
        self.publish_rate    = self.get_parameter('publish_rate').value
        self.horizon         = int(self.get_parameter('horizon').value)
        self.simulation_time = float(self.get_parameter('simulation_time').value)
        self.goal_tolerance  = float(self.get_parameter('goal_tolerance').value)

        self.current_pose          = np.zeros(3)
        self.current_pose_received = False
        self.global_path           = None
        # tube 명목 오차 상태: 사이클 간 전파됨. 목표/경로가 갱신되면 None으로 리셋해
        # 다음 사이클에서 실측 오차로 재정렬한다.
        self.e_nom                 = None

        # /odom: EKF가 출력하는 융합 odom (바퀴 + IMU)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odometry_callback, 10,
        )

        if self.use_goal_topic:
            self.goal_sub = self.create_subscription(
                PoseStamped, 'mpc_goal', self.goal_callback, 10,
            )

        if self.use_global_path:
            self.path_sub = self.create_subscription(
                Path, 'global_path', self.global_path_callback, 10,
            )

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer   = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.mpc = self._create_mpc_planner()
        self.get_logger().info('MPC Tube bridge initialized.')

    def odometry_callback(self, msg: Odometry):
        self.current_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self._quaternion_to_yaw(msg.pose.pose.orientation),
        ], dtype=float)
        self.current_pose_received = True

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            self._quaternion_to_yaw(msg.pose.orientation),
        ], dtype=float)
        self.e_nom = None   # 새 목표 → 명목 상태 재정렬
        self.get_logger().info(f'Goal updated: {self.goal_pose}')

    def global_path_callback(self, msg: Path):
        if not msg.poses:
            return
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
        if not self.current_pose_received:
            return

        current = self.current_pose.copy()

        # 최종 목표 도달 판정 → 정지
        if self.use_global_path and self.global_path is not None and len(self.global_path) > 0:
            final_goal = self.global_path[-1]
        else:
            final_goal = self.goal_pose
        if np.hypot(final_goal[0] - current[0], final_goal[1] - current[1]) < self.goal_tolerance:
            self.cmd_pub.publish(Twist())   # zero stop
            return

        # 참조 궤적(world 프레임) + 참조 입력(uRef: 피드포워드) 생성
        if self.use_global_path and self.global_path is not None and len(self.global_path) > 1:
            qRef, uRef = self._generate_reference_from_path(current, self.global_path)
        else:
            qRef, uRef = self._generate_reference(current, self.goal_pose.copy())

        # 실측 추종오차(로봇 프레임): 참조점 qRef[:,0] 대비 현재 pose
        e_act = self.mpc.compute_error(current, qRef[:, 0])

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
            self.get_logger().warn(f'MPC QP failed: {e}')
            self.cmd_pub.publish(Twist())
            return

        # Tube ancillary: 실측-명목 편차를 K로 되먹임해 tube 안에 가둠
        u_corr = u_nom - self.mpc.K @ (e_act - self.e_nom)

        # 명목 상태 1스텝 전파 → 다음 사이클로 이어짐 (tube 핵심)
        self.e_nom = A0 @ self.e_nom + self.mpc.B @ u_nom

        # 최종 명령 = 피드포워드(참조 입력) + 보정량, 속도 한계로 클램프
        v_cmd = clamp(uRef[0, 0] + u_corr[0], -self.max_v, self.max_v)
        w_cmd = clamp(uRef[1, 0] + u_corr[1], -self.max_w, self.max_w)

        cmd           = Twist()
        cmd.linear.x  = float(v_cmd)
        cmd.angular.z = float(w_cmd)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'pose={current.round(2)}, e_act={e_act.round(3)}, cmd=[{v_cmd:.3f}, {w_cmd:.3f}]')

    def _create_mpc_planner(self):
        """TubeMPCPlanner 초기화: 상태/입력 제약 집합, LQR gain, B행렬 설정."""
        Ts    = 0.1
        x_min = np.array([-2.0, -2.0, -0.3])   # 허용 오차 범위 (x_err, y_err, θ_err)
        u_min = np.array([-0.5, -0.5])           # 최소 제어입력 (v, ω)
        w_min = np.array([-0.1, -0.1, -0.1])     # 허용 외란 범위
        e_min = np.array([-0.2, -0.2, -0.2])     # tube 크기

        v0 = 0.05
        w0 = 0.0
        A  = self._A_matrix(v0, w0)
        B  = np.array([[Ts, 0.0], [0.0, 0.0], [0.0, Ts]])
        Q  = 100 * np.eye(3)   # 상태 오차 가중치 (클수록 경로 추종 우선)
        R  = 0.01 * np.eye(2)  # 입력 가중치 (클수록 부드러운 입력 우선)
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

        idx        = closest
        qRef[:, 0] = self._path_pose(pts, idx)
        for i in range(self.horizon):
            idx            = self._advance_along_path(pts, idx, step_len)
            qRef[:, i + 1] = self._path_pose(pts, idx)
            dx = qRef[0, i + 1] - qRef[0, i]
            dy = qRef[1, i + 1] - qRef[1, i]
            v_ref = float(np.hypot(dx, dy)) / 0.1
            w_ref = wrap_angle(qRef[2, i + 1] - qRef[2, i]) / 0.1
            uRef[:, i] = [clamp(v_ref, 0.0, self.max_v),
                          clamp(w_ref, -self.max_w, self.max_w)]

        self.ref_u = uRef.copy()
        return qRef, uRef

    def _path_pose(self, pts, idx):
        """경로점 idx의 [x, y, heading]. heading 은 인접 점으로의 접선."""
        nxt = min(idx + 1, len(pts) - 1)
        prv = idx if nxt != idx else max(idx - 1, 0)
        heading = math.atan2(pts[nxt, 1] - pts[prv, 1], pts[nxt, 0] - pts[prv, 0])
        return np.array([pts[idx, 0], pts[idx, 1], heading], dtype=float)

    def _advance_along_path(self, pts, idx, distance):
        """idx에서 경로를 따라 distance(m)만큼 전진한 점의 인덱스."""
        acc = 0.0
        j   = idx
        while j + 1 < len(pts) and acc < distance:
            acc += float(np.hypot(pts[j + 1, 0] - pts[j, 0], pts[j + 1, 1] - pts[j, 1]))
            j   += 1
        return j

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
