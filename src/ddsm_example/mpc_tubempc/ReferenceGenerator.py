import math
import numpy as np


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def generate_reference(start, goal, Np, Ts, max_v=0.2, max_w=0.5):
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)

    delta = goal - start
    distance = float(np.hypot(delta[0], delta[1]))
    heading_to_goal = math.atan2(delta[1], delta[0])
    heading_error = wrap_angle(heading_to_goal - start[2])

    v_nom = min(max_v, distance / (Np * Ts) if distance > 0 else 0.0)
    w_nom = min(max_w, abs(heading_error) / (Np * Ts)) * np.sign(heading_error)

    qRef = np.zeros((3, Np + 1))
    uRef = np.zeros((2, Np))
    qRef[:, 0] = start

    for i in range(Np):
        uRef[:, i] = [v_nom, w_nom]
        q = qRef[:, i]
        qRef[0, i + 1] = q[0] + v_nom * Ts * math.cos(q[2])
        qRef[1, i + 1] = q[1] + v_nom * Ts * math.sin(q[2])
        qRef[2, i + 1] = wrap_angle(q[2] + w_nom * Ts)

    return qRef, uRef
