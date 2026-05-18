import sys
from pathlib import Path
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

# 기존 mpc_tubempc 폴더를 import 경로에 추가합니다.
this_dir = Path(__file__).resolve().parent
mpc_dir = this_dir.parents[2] / 'ddsm_example' / 'mpc_tubempc'
if str(mpc_dir) not in sys.path:
    sys.path.insert(0, str(mpc_dir))

from TubeMPCPlanner import TubeMPCPlanner


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)


class MPCBridgeNode(Node):
    def __init__(self):
        super().__init__('mpc_tubempc_bridge')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('goal_x', 0.0),
                ('goal_y', 0.0),
                ('goal_theta', 0.0),
                ('use_goal_topic', False),
                ('use_global_path', False),
                ('velocity_limit', 0.2),
                ('omega_limit', 1.0),
                ('publish_rate', 10.0),
                ('horizon', 6),
                ('simulation_time', 20.0),
            ]
        )

        self.goal_pose = np.array([
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value,
            self.get_parameter('goal_theta').value,
        ], dtype=float)
        self.use_goal_topic = self.get_parameter('use_goal_topic').value
        self.use_global_path = self.get_parameter('use_global_path').value
        self.max_v = self.get_parameter('velocity_limit').value
        self.max_w = self.get_parameter('omega_limit').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.horizon = int(self.get_parameter('horizon').value)
        self.simulation_time = float(self.get_parameter('simulation_time').value)

        self.current_pose = np.zeros(3)
        self.current_pose_received = False
        self.global_path = None

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10,
        )

        if self.use_goal_topic:
            self.goal_sub = self.create_subscription(
                PoseStamped,
                'mpc_goal',
                self.goal_callback,
                10,
            )

        if self.use_global_path:
            self.path_sub = self.create_subscription(
                Path,
                'global_path',
                self.global_path_callback,
                10,
            )

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

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
        self.get_logger().info(f'Goal updated: {self.goal_pose}')

    def global_path_callback(self, msg: Path):
        if not msg.poses:
            return
        self.global_path = [
            np.array([pose.pose.position.x, pose.pose.position.y, self._quaternion_to_yaw(pose.pose.orientation)], dtype=float)
            for pose in msg.poses
        ]
        self.get_logger().info(f'Received global path with {len(self.global_path)} points.')

    def timer_callback(self):
        if not self.current_pose_received:
            return

        current = self.current_pose.copy()
        if self.use_global_path and self.global_path is not None and len(self.global_path) > 1:
            qRef, uRef = self._generate_reference_from_path(current, self.global_path)
            goal = self.global_path[-1]
        else:
            goal = self.goal_pose.copy()
            qRef, uRef = self._generate_reference(current, goal)

        q = current
        e_k = self.mpc.compute_error(q, qRef[:, 0])

        A0 = self._A_matrix(uRef[0, 0], uRef[1, 0])
        A1 = self._A_matrix(uRef[0, 1], uRef[1, 1]) if self.horizon > 1 else A0
        A2 = self._A_matrix(uRef[0, 2], uRef[1, 2]) if self.horizon > 2 else A0
        A3 = self._A_matrix(uRef[0, 3], uRef[1, 3]) if self.horizon > 3 else A0

        B_bar, A_bar = self.mpc.construct_augmentemd_model(A0, A1, A2, A3)
        Uad_A, Uad_b = self.mpc.construct_constraint_matrices(B_bar, A0, A1, A2, A3, e_k)
        H_qp, f_qp = self.mpc.construct_cost_matrices(B_bar, A_bar, e_k)

        try:
            u_mpc = self.mpc.solve_qp(H_qp, f_qp, Uad_A, Uad_b)
        except Exception as e:
            self.get_logger().warn(f'MPC QP failed: {e}')
            return

        u_act = u_mpc - self.mpc.K @ (self.mpc.e_act[:, 0] - self.mpc.e_nom[:, 0])
        u_act[0] = clamp(u_act[0], -self.max_v, self.max_v)
        u_act[1] = clamp(u_act[1], -self.max_w, self.max_w)

        self.mpc.e_nom[:, 1] = A0 @ e_k + self.mpc.B @ u_mpc
        self.mpc.e_act[:, 1] = A0 @ e_k + self.mpc.B @ u_act
        self.mpc.U_act[:, 0] = u_act
        self.mpc.U_nom[:, 0] = u_mpc

        cmd = Twist()
        cmd.linear.x = float(u_act[0])
        cmd.angular.z = float(u_act[1])
        self.cmd_pub.publish(cmd)

        self.get_logger().info(f'goal={goal.round(2)}, pose={current.round(2)}, u_act={u_act.round(3)}')

    def _create_mpc_planner(self):
        Ts = 0.1
        x_min = np.array([-2.0, -2.0, -0.3])
        u_min = np.array([-0.5, -0.5])
        w_min = np.array([-0.1, -0.1, -0.1])
        e_min = np.array([-0.2, -0.2, -0.2])

        v0 = 0.05
        w0 = 0.0
        A = self._A_matrix(v0, w0)
        B = np.array([[Ts, 0.0], [0.0, 0.0], [0.0, Ts]])
        Q = 100 * np.eye(3)
        R = 0.01 * np.eye(2)
        P = self._solve_are(A, B, Q, R)
        K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)

        return TubeMPCPlanner(Ts, self.simulation_time, self.horizon, x_min, u_min, w_min, e_min, K, B, self._A_matrix_from_ref)

    def _A_matrix(self, v, w):
        Ts = 0.1
        return np.array([
            [1.0, Ts * w, 0.0],
            [-Ts * w, 1.0, Ts * v],
            [0.0, 0.0, 1.0],
        ])

    def _A_matrix_from_ref(self, k, offset):
        v = self.ref_u[0, min(k + offset, self.horizon - 1)]
        w = self.ref_u[1, min(k + offset, self.horizon - 1)]
        return self._A_matrix(v, w)

    def _generate_reference(self, current, goal):
        delta = goal - current
        dist = float(np.hypot(delta[0], delta[1]))
        heading_to_goal = math.atan2(delta[1], delta[0])
        heading_error = wrap_angle(heading_to_goal - current[2])

        v_nom = clamp(dist / (self.horizon * 0.1), 0.0, self.max_v)
        w_nom = clamp(heading_error / (self.horizon * 0.1), -self.max_w, self.max_w)

        qRef = np.zeros((3, self.horizon + 1))
        uRef = np.zeros((2, self.horizon))
        qRef[:, 0] = current
        self.ref_u = np.zeros((2, self.horizon))

        for i in range(self.horizon):
            self.ref_u[:, i] = [v_nom, w_nom]
            q = qRef[:, i]
            qRef[0, i + 1] = q[0] + v_nom * 0.1 * math.cos(q[2])
            qRef[1, i + 1] = q[1] + v_nom * 0.1 * math.sin(q[2])
            qRef[2, i + 1] = wrap_angle(q[2] + w_nom * 0.1)

        uRef[:] = self.ref_u
        return qRef, uRef

    def _generate_reference_from_path(self, current, path):
        qRef = np.zeros((3, self.horizon + 1))
        uRef = np.zeros((2, self.horizon))
        qRef[:, 0] = current
        self.ref_u = np.zeros((2, self.horizon))

        for i in range(self.horizon):
            target_index = min(i + 1, len(path) - 1)
            target = path[target_index]
            delta = target - qRef[:, i]
            dist = float(np.hypot(delta[0], delta[1]))
            heading = math.atan2(delta[1], delta[0])
            heading_error = wrap_angle(heading - qRef[2, i])

            v = clamp(dist / 0.1, 0.0, self.max_v)
            w = clamp(heading_error / 0.1, -self.max_w, self.max_w)

            uRef[:, i] = [v, w]
            qRef[0, i + 1] = qRef[0, i] + v * 0.1 * math.cos(qRef[2, i])
            qRef[1, i + 1] = qRef[1, i] + v * 0.1 * math.sin(qRef[2, i])
            qRef[2, i + 1] = wrap_angle(qRef[2, i] + w * 0.1)

        return qRef, uRef

    def _solve_are(self, A, B, Q, R):
        from scipy.linalg import solve_discrete_are
        return solve_discrete_are(A, B, Q, R)

    def _quaternion_to_yaw(self, q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


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
