import time
import polytope as pc
import numpy as np
import polytope as pc
import matplotlib.pyplot as plt
import cvxpy as cp

from scipy.linalg import solve_discrete_are
from scipy.optimize import minimize
from scipy.linalg import block_diag

from ReferenceGenerator import generate_reference
from TubeMPCPlanner import TubeMPCPlanner
from communication import CommunicationHandler

# 파라미터
Ts =0.1
Simulation_time = 30
amp, a, b =2 , 2, 1
Nsim = np.arange(0,Simulation_time,Ts)
Np =4

# 2) TubeMPCPlanner 초기화
# — 여기에 set_x, set_u, set_e, K, B, A_matrix_func 를 미리 정의

x_min = np.array([-1.5,-1.5,-1.5])
x_max = -x_min
u_min = np.array([-5 ,-5])
u_max = -u_min
w_min = np.array([-0.2, -0.2, -0.2])
w_max = -w_min
e_min = np.array([-0.05, -0.05, -0.05])
e_max = -e_min

set_x = pc.Polytope(np.vstack((np.eye(3), -np.eye(3))), np.hstack((x_max, -x_min)))
set_u = pc.Polytope(np.vstack((np.eye(2), -np.eye(2))), np.hstack((u_max, -u_min)))
set_w = pc.Polytope(np.vstack((np.eye(3), -np.eye(3))), np.hstack((w_max, -w_min)))
set_e = pc.Polytope(np.vstack((np.eye(3), -np.eye(3))), np.hstack((e_max, -e_min)))

t,qRef,uRef = generate_reference()

v0,w0 = uRef[:,0]

# === 시스템 행렬 정의 ===
A = np.array([
    [1, Ts * w0, 0],
    [-Ts * w0 , 1, Ts * v0 ],
    [0, 0, 1]
])
B = np.array([
    [Ts, 0],
    [0, 0],
    [0, Ts]
])

# === LQR 설계 ===
Q = 100 * np.eye(3)
R = 0.01 * np.eye(2)

# LQR 계산
P = solve_discrete_are(A, B, Q, R)
K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)
A_cl = A - B @ K

def A_matrix_func(k, offset):
    v = uRef[0, k+offset]  # ← 0번째 행 (v)
    w = uRef[1, k+offset]   # ← 1번째 행 (w)
    return np.array([
        [1, Ts*w,       0],
        [-Ts*w, 1, Ts*v  ],
        [0,     0,       1]
    ])

q =np.array([0,0,0]).reshape(-1) # 강제로 (3,)로 즉 1D 차원

# === 외란 초기화 (time-varying disturbance 확인 목적) ===
w_input = np.zeros((2, len(Nsim)))
for k in range(len(Nsim)-1):
    d = 2
    w_noise = 0  # 평균 0, 표준편차 0.01의 가우시안 노이즈 (지금은 꺼둠)
    disturbance = np.array([
        d * 0.1 * np.sin(0.1 * k),
        d * 0.1 * np.sin(0.1 * k)
    ])
    w_input[:, k] = disturbance + w_noise

# 3) Communication 핸들러 초기화
comm = CommunicationHandler('192.168.0.69', 5000)
comm.reset()  # 위치·시간 기준 초기화

# 4) 데이터 기록용
x_log, y_log, th_log, time_log = [], [], [], []

mpc = TubeMPCPlanner(Ts,Simulation_time,Np,x_min,u_min,w_min,e_min,K,B,A_matrix_func)


e_act = mpc.e_act[:,1]
e_nom = mpc.e_nom[:,1]
U_act = mpc.U_act[:,1]
U_nom = mpc.U_nom[:,1]

print(len(Nsim))

# 5) 메인 루프
start_time = time.time()

# 6) 초기위치 갱신해주자
prev_q =np.array([0,0,0]).reshape(-1) # 강제로 (3,)로 즉 1D 차원


for k in range(len(Nsim)-1):

    # 1) 먼저 피드백으로 q 확정 (없으면 prev_q 유지 or 스킵)
    fb = comm.get_feedback()  # (x,y,theta) or None / none 이거나 3개(x y theta)가 아닐 시에는 그냥 스킵
    if not fb or len(fb) != 3:
        print("⏰ Timeout: keep last q")
        q = prev_q.copy()
        # time.sleep(Ts); continue  # 스킵할거면 이 라인 사용
    else:
        try:
            x, y, theta = fb
            q = np.array([float(x), float(y), float(theta)], dtype=float)
            # q[2] = np.deg2rad(q[2])  # 만약 도(deg)라면 라디안 변환
            prev_q = q.copy()
        except Exception:
            print("⚠️ parse fail: keep last q")
            q = prev_q.copy()

    q_k_ref = qRef[:,k]
    e_k = mpc.compute_error(q,q_k_ref)

    #  A 시퀀스 생성
    A0 = A_matrix_func(k,0)
    A1 = A_matrix_func(k,1)
    A2 = A_matrix_func(k,2)
    A3 = A_matrix_func(k,3)

    # 5.2.4) 증강 모델 구성
    B_bar, A_bar = mpc.construct_augmentemd_model(A0,A1,A2,A3)
    # 5.2.5) 제약 매트릭스
    Uad_A, Uad_b = mpc.construct_constraint_matrices(B_bar,A0,A1,A2,A3,e_k)
    # 5.2.6) 비용행렬
    H_qp, f_qp = mpc.construct_cost_matrices(B_bar, A_bar, e_k)

    # 5.2.7) QP 풀기 → u_mpc
    u_mpc = mpc.solve_qp(H_qp, f_qp, Uad_A, Uad_b)

    # 5.3) 첫 스텝 보상 제어 입력
    u_act = u_mpc - K@(mpc.e_act[:,k]-mpc.e_nom[:,k])
    
    mpc.e_nom[:, k+1] = A0 @ e_k + B @ u_mpc
    mpc.e_act[:, k+1] = A0 @ e_k + B @ (u_act + w_input[:, k])
    mpc.U_act[:, k] = u_act
    mpc.U_nom[:, k] = u_mpc

    #
    v_cmd, w_cmd = u_act[0], u_act[1]

    # 5.4) 속도 명령 전송
    comm.send_command(v_cmd, w_cmd)

    # 5.4-1) 피드백  / -1 (쩡국)
    #-1 [x,y,theta] = comm.get_feedback()
    #-1 q= np.array([x,y,theta]).reshape(-1)# 강제로 (3,)로 즉 1D 차원
    x, y, theta = q  # 이미 확정된 q 사용
    print(f"[{k}] u_mpc = {u_mpc}, u_act = {u_act}, q = {q}, v = {v_cmd:.2f}, w = {w_cmd:.2f}")

    # 5.5) 로그 기록
    x_log.append(x)
    y_log.append(y)
    th_log.append(theta)
    time_log.append(time.time() - start_time)

    # 5.6) 종료 조건
    if time.time() - start_time > Simulation_time:
        break

# 6) 최종 정지
comm.send_command(0.0, 0.0)

# 7) 결과 출력 (간단 예시)
import matplotlib.pyplot as plt
plt.figure()
plt.plot(x_log, y_log, 'r-', label='Actual')
plt.plot(qRef[0,:len(x_log)], qRef[1,:len(x_log)], 'b--', label='Reference')
plt.legend()
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Tube-MPC Trajectory Tracking')
plt.show()

