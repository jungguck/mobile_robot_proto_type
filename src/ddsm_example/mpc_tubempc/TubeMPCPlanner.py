import numpy as np
import polytope as pc
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are
from scipy.optimize import minimize
from scipy.linalg import block_diag
import cvxpy as cp


class TubeMPCPlanner:
    def __init__(self, Ts, Simulation_time, Np, x_min, u_min,w_min ,e_min, K, B, A_matrix_func):
        self.Ts = Ts
        self.Np = Np
        self.K = K
        self.x_min = x_min
        self.u_min = u_min
        self.e_min = e_min
        self.w_min = w_min
        self.x_max = -x_min
        self.u_max = -u_min
        self.e_max = -e_min
        self.w_max = -w_min

        self.set_x =  pc.Polytope(np.vstack((np.eye(3), -np.eye(3))), np.hstack((self.x_max, -self.x_min)))
        self.set_u =  pc.Polytope(np.vstack((np.eye(2), -np.eye(2))), np.hstack((self.u_max, -self.u_min)))
        self.set_e =  pc.Polytope(np.vstack((np.eye(3), -np.eye(3))), np.hstack((self.e_max, -self.e_min)))
        self.set_w =  pc.Polytope(np.vstack((np.eye(3), -np.eye(3))), np.hstack((self.w_max, -self.w_min)))

        self.x_A = self.set_x.A
        self.x_b = self.set_x.b - np.abs(self.set_e.A @ self.e_min.reshape(-1, 1))
        self.set_xbar = pc.Polytope(self.x_A, self.x_b)

        self.K_e_min = self.K @ self.e_min # @는 행렬곱을 의미 K(2x3) / e(3x1)
        self.K_e_max = self.K @ self.e_max
        self.K_e_bound = np.maximum(np.abs(self.K_e_min), np.abs(self.K_e_max))

        self.u_A = self.set_u.A
        self.u_b = self.set_u.b - np.abs(self.set_u.A @ self.K_e_bound.reshape(-1, 1))
        self.set_ubar = pc.Polytope(self.u_A, self.u_b)

        self.x_A = self.set_xbar.A
        self.x_b = self.set_xbar.b
        self.u_A = self.set_ubar.A
        self.u_b = self.set_ubar.b

        self.B = B
        self.A_matrix_func = A_matrix_func

        self.simn = int(float(Simulation_time) / float(Ts))

        self.nx = self.B.shape[0]
        self.nu = self.B.shape[1]

        self.e_nom = np.zeros((self.nx, self.simn))
        self.e_act = np.zeros((self.nx, self.simn))    
        self.U_act = np.full((self.nu, self.simn), np.nan)
        self.U_nom = np.full((self.nu, self.simn), np.nan)

        # Fr: Exponential decay for e_ref
        ar = 0.8
        Ar = ar * np.eye(self.nx)
        self.Fr = np.hstack([np.linalg.matrix_power(Ar, i + 1) for i in range(self.Np)]).T

    def compute_error(self, q, qRef_k):
        # 안전하게 float 변환
        q = np.asarray(q, dtype=float)
        qRef_k = np.asarray(qRef_k, dtype=float)

        # None/NaN 검사
        if np.any(np.isnan(q)) or np.any(np.isnan(qRef_k)):
            raise ValueError(f"Invalid q or qRef_k: q={q}, qRef_k={qRef_k}")

        # 상태 분리
        x, y, theta = q

        # 회전 행렬
        rot = np.array([
            [np.cos(theta),  np.sin(theta), 0],
            [-np.sin(theta), np.cos(theta), 0],
            [0,              0,             1]
        ])
        # 오차 계산
        e = rot @ (qRef_k - q)
        # heading 오차를 [-π, π]로 래핑
        e[2] = (e[2] + np.pi) % (2 * np.pi) - np.pi

        return e

    def construct_augmentemd_model(self, A0,A1,A2,A3):
        Z = np.zeros((self.nx, self.nu))

        B_bar = np.vstack([
            np.hstack([self.B, Z, Z, Z]),
            np.hstack([A0 @ self.B, self.B, Z, Z]),
            np.hstack([A0 @ A1 @ self.B, A0 @ self.B, self.B, Z]),
            np.hstack([A0 @ A1 @ A2 @ self.B, A0 @ A1 @ self.B, A0 @ self.B, self.B])
        ])

        A_bar = np.vstack([A0, 
                           A0 @ A1, 
                           A0 @ A1 @ A2, 
                           A0 @ A1 @ A2 @ A3
                           ])

        return B_bar, A_bar

    def construct_constraint_matrices(self, B_bar, A0,A1,A2,A3, e):


        F_ext = np.kron(np.eye(self.Np), self.x_A)
        G = np.kron(np.eye(self.Np), self.u_A)
        g_tilda = np.kron(np.ones(self.Np), self.u_b)
        f_tilda = np.kron(np.ones(self.Np), self.x_b)

        FA = np.vstack([
            self.x_A @ A0 ,
            self.x_A @ A0 @ A1 ,
            self.x_A @ A0 @ A1 @ A2 ,
            self.x_A @ A0 @ A1 @ A2 @ A3 
        ]) @ e

        F_A = F_ext @ B_bar
        F_B = f_tilda - FA

        Uad_A = np.vstack([F_A, G])
        Uad_b = np.hstack([F_B.flatten(), g_tilda])

        return Uad_A, Uad_b

    def construct_cost_matrices(self, B_bar, A_bar, e):
        Qt = block_diag(*([np.diag([100, 100, 100])] * self.Np))
        Rt = block_diag(*([np.diag([0.1, 0.1])] * self.Np))

        H_qp = B_bar.T @ Qt @ B_bar + Rt
        f_qp = -2 * B_bar.T @ Qt @ (A_bar - self.Fr) @ e
        return H_qp, f_qp

    def solve_qp(self, H_qp, f_qp, Uad_A, Uad_b):
        u_var = cp.Variable(self.nu * self.Np)

        objective = 0.5 * cp.quad_form(u_var, H_qp) + f_qp.T @ u_var
        constraints = [Uad_A @ u_var <= Uad_b]

        prob = cp.Problem(cp.Minimize(objective), constraints)
        prob.solve(solver=cp.OSQP)

        if u_var.value is None:
            raise ValueError("QP solver failed to find a solution.")

        return u_var.value[:self.nu]  # 첫 스텝 제어입력 반환