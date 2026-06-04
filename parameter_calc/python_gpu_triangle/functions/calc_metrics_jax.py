"""
calc_metrics_jax: 주행 품질 지표 — 벡터화 버전

이미 원본이 벡터 연산 위주이므로 거의 동일하게 유지.
JAX 의존성 없이 numpy로 충분히 빠름.
"""
import numpy as np


def calc_metrics_gpu(x_arr, x_t, y_t, R, p):
    N = len(x_arr)

    y_wr = np.interp(R['xwr'], x_t, y_t)
    y_wm = np.interp(R['xwm'], x_t, y_t)
    y_wf = np.interp(R['xwf'], x_t, y_t)
    y_avg = (y_wr + y_wm + y_wf) / 3

    idx_flat = (x_arr >= (x_arr[0] + 0.1)) & (x_arr <= (x_arr[0] + 0.6))
    if np.any(idx_flat):
        h_offset_ref = np.nanmean(R['y0'][idx_flat] - y_avg[idx_flat])
    else:
        h_offset_ref = R['y0'][0] - y_avg[0]

    y_ideal_v4 = y_avg + h_offset_ref
    err_v4 = R['y0'] - y_ideal_v4

    if 'ok' in R:
        err_v4[~R['ok']] = 0.5

    MSD = max(np.nanmean(err_v4**2), 1e-20)

    win_v2 = max(3, min(40, N // 4))
    y_ideal_v2 = np.convolve(R['y0'], np.ones(win_v2) / win_v2, mode='same')
    mse_v2 = max(np.nanmean((R['y0'] - y_ideal_v2)**2), 1e-20)

    valid_mask = R['ok'] if 'ok' in R else ~np.isnan(R['y0'])

    return {
        'SN_dB': -10 * np.log10(MSD),
        'err_std': np.sqrt(MSD),
        'SN_dB_v2': -10 * np.log10(mse_v2),
        'h_std': float(np.nanstd(R['y0'])),
        'r_range_deg': float(np.rad2deg(np.nanmax(R['ar']) - np.nanmin(R['ar']))),
        'b_range_deg': float(np.rad2deg(np.nanmax(R['bb']) - np.nanmin(R['bb']))),
        'y0_valid': R['y0'][valid_mask],
        'y_ideal': y_ideal_v4[valid_mask],
    }
