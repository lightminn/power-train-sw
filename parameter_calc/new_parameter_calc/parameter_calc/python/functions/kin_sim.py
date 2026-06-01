"""
kin_sim: 통합 역기구학 시뮬레이션 [v4]
"""
import numpy as np
from scipy.optimize import fsolve
from scipy.interpolate import interp1d
from .ceq import ceq
from .wpos import wpos


def _fill_nan(v):
    idx = np.arange(len(v))
    vld = ~np.isnan(v)
    if np.sum(vld) < 2:
        v[~vld] = 0
        return v
    f = interp1d(idx[vld], v[vld], kind='nearest', fill_value='extrapolate')
    return f(idx)


def kin_sim(x_arr, x_t, y_t, p):
    x_arr = np.atleast_1d(x_arr)
    N = len(x_arr)

    R = {
        'y0': np.full(N, np.nan), 'ar': np.full(N, np.nan), 'bb': np.full(N, np.nan),
        'xwf': np.full(N, np.nan), 'ywf': np.full(N, np.nan),
        'xwm': np.full(N, np.nan), 'ywm': np.full(N, np.nan),
        'xwr': np.full(N, np.nan), 'ywr': np.full(N, np.nan),
        'xpb': np.full(N, np.nan), 'ypb': np.full(N, np.nan),
        'xcg': np.full(N, np.nan), 'ycg': np.full(N, np.nan),
        'ok': np.ones(N, dtype=bool),
    }

    # bb 초기값 결정
    bmode = p.get('bogie_mode', p.get('bogie_type', 'legacy')).lower()

    if bmode == 'triangle':
        bb0 = p.get('beta_b', np.deg2rad(90)) / 2
    elif bmode in ('frame', 'trapezoid'):
        bb0 = 0.0
    else:
        bb0 = 0.0

    X0 = np.array([p['R_w'], 0.0, bb0])
    X0_fixed = X0.copy()

    F_terrain = interp1d(x_t, y_t, kind='linear', fill_value=(y_t[0], y_t[-1]),
                         bounds_error=False)

    for i in range(N):
        xb = x_arr[i]
        fun = lambda X: ceq(X, xb, F_terrain, p)

        sol, info, ier, msg = fsolve(fun, X0, full_output=True)
        ef = ier

        if ef != 1:
            sol2, info2, ier2, msg2 = fsolve(fun, X0_fixed, full_output=True)
            if ier2 == 1:
                sol = sol2
                ef = ier2

        if ef == 1:
            X0 = sol.copy()
            R['y0'][i] = sol[0]
            R['ar'][i] = sol[1]
            R['bb'][i] = sol[2]
            Wf, Wm, Wr, Pb, CG = wpos(sol, xb, p)
            R['xwf'][i] = Wf[0]; R['ywf'][i] = Wf[1]
            R['xwm'][i] = Wm[0]; R['ywm'][i] = Wm[1]
            R['xwr'][i] = Wr[0]; R['ywr'][i] = Wr[1]
            R['xpb'][i] = Pb[0]; R['ypb'][i] = Pb[1]
            R['xcg'][i] = CG[0]; R['ycg'][i] = CG[1]
        else:
            R['ok'][i] = False

    n_fail = np.sum(~R['ok'])
    R['fail_rate'] = n_fail / N

    if 0 < n_fail < N:
        R['y0'] = _fill_nan(R['y0'])
        R['ar'] = _fill_nan(R['ar'])
        R['bb'] = _fill_nan(R['bb'])
        fi = np.where(~R['ok'])[0]
        n_recovered = 0

        for i in fi:
            xb_i = x_arr[i]
            X0_i = np.array([R['y0'][i], R['ar'][i], R['bb'][i]])
            fun_i = lambda X: ceq(X, xb_i, F_terrain, p)
            sol_r, info_r, ier_r, _ = fsolve(fun_i, X0_i, full_output=True)
            if ier_r == 1:
                R['y0'][i] = sol_r[0]
                R['ar'][i] = sol_r[1]
                R['bb'][i] = sol_r[2]
                R['ok'][i] = True
                n_recovered += 1
            sol_use = np.array([R['y0'][i], R['ar'][i], R['bb'][i]])
            Wf, Wm, Wr, Pb, CG = wpos(sol_use, xb_i, p)
            R['xwf'][i] = Wf[0]; R['ywf'][i] = Wf[1]
            R['xwm'][i] = Wm[0]; R['ywm'][i] = Wm[1]
            R['xwr'][i] = Wr[0]; R['ywr'][i] = Wr[1]
            R['xpb'][i] = Pb[0]; R['ypb'][i] = Pb[1]
            R['xcg'][i] = CG[0]; R['ycg'][i] = CG[1]

        R['fail_rate'] = np.sum(~R['ok']) / N
        print(f'  [보간+재시도] {n_fail}/{N} 포인트 실패 → {n_recovered}개 복원')

    elif n_fail == N:
        raise RuntimeError(
            f'kin_sim: 모든 포인트 수렴 실패. bogie_mode="{bmode}", R_w={p["R_w"]:.3f}'
        )

    R['fail_idx'] = np.where(~R['ok'])[0]
    return R
