"""
calc_envelope: Minkowski Sum 기법을 이용한 지형 팽창 (Wheel Center Envelope)
"""
import numpy as np


def calc_envelope(x_t, y_t, R_w):
    N = len(x_t)
    y_env = y_t + R_w
    dx = x_t[1] - x_t[0]
    n_radius = int(np.ceil(R_w / dx))

    for i in range(N):
        idx_start = max(0, i - n_radius)
        idx_end = min(N, i + n_radius + 1)
        x_win = x_t[idx_start:idx_end]
        y_win = y_t[idx_start:idx_end]

        dy2 = R_w**2 - (x_t[i] - x_win)**2
        dy2 = np.maximum(dy2, 0)
        y_candidates = y_win + np.sqrt(dy2)

        y_env[i] = np.max(y_candidates)

    return y_env
