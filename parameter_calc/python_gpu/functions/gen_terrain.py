"""
gen_terrain: 지형 생성 함수 (통합 단일 버전)

지원 타입:
    'flat'        평지
    'step'        단일 단차
    'stairs'      4단 계단
    'rough'       불규칙 연속 지형 (사인파 합성)
    'real_stairs' 실제 계단 스펙
    'wood_block'  90×90mm 목재 블록 이산 노면
"""
import numpy as np


def gen_terrain(terrain_type, p):
    L = 10
    N = 8000

    x_t = np.linspace(0, L, N)
    y_t = np.zeros(N)

    t = terrain_type.lower()

    if t == 'flat':
        pass

    elif t == 'step':
        y_t[x_t >= L / 2] = p['obs_h']

    elif t == 'stairs':
        n_step = 4
        dh = p['obs_h'] / n_step
        dw = 0.25
        x0 = 2.0
        for k in range(1, n_step + 1):
            y_t[x_t >= x0 + (k - 1) * dw] = k * dh

    elif t == 'rough':
        a = 0.025
        y_t = (a * np.sin(2 * np.pi * x_t / 0.9)
               + a * 0.6 * np.cos(2 * np.pi * x_t / 0.4 + 0.8)
               + a * 0.4 * np.sin(2 * np.pi * x_t / 1.5 + 2.1))
        y_t = y_t - np.min(y_t)

    elif t == 'real_stairs':
        rise = 0.080
        depth = 0.500
        n_steps = 3
        x0_up = 1.5

        for k in range(1, n_steps + 1):
            y_t[x_t >= x0_up + (k - 1) * depth] = k * rise

        x_top = x0_up + n_steps * depth
        x_top_end = x_top + 1.5

        x0_dn = x_top_end
        plateau_h = n_steps * rise

        y_t[(x_t >= x_top) & (x_t < x_top_end)] = plateau_h

        for k in range(1, n_steps + 1):
            x_edge = x0_dn + (k - 1) * depth
            h_level = plateau_h - k * rise
            y_t[x_t >= x_edge] = max(h_level, 0)

        y_t[x_t >= x0_dn + n_steps * depth] = 0
        y_t = np.maximum(y_t, 0)

    elif t == 'wood_block':
        block_w = 0.090
        h_levels = [0.040, 0.050, 0.060, 0.070, 0.080]

        x_wood_start = 1.5
        x_wood_end = L - 1.5

        rng = np.random.RandomState(42)

        block_positions = np.arange(x_wood_start, x_wood_end - block_w, block_w)

        for curr_x in block_positions:
            h_cur = h_levels[rng.randint(len(h_levels))]
            idx = (x_t >= curr_x) & (x_t < curr_x + block_w)
            y_t[idx] = h_cur

        y_t[x_t < x_wood_start] = 0
        y_t[x_t >= x_wood_end] = 0

    else:
        raise ValueError(
            f'gen_terrain: 알 수 없는 지형 타입 "{terrain_type}"\n'
            '사용 가능: flat, step, stairs, rough, real_stairs, wood_block'
        )

    return x_t, y_t
