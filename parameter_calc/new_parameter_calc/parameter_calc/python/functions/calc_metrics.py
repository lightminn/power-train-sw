"""
calc_metrics: 주행 품질 지표 계산 [v4]
"""
import numpy as np


def calc_metrics(x_arr, x_t, y_t, R, p):
    N = len(x_arr)

    if 'R_w' not in p:
        raise ValueError('calc_metrics: 파라미터 구조체 p에 R_w 값이 누락되었습니다.')

    # v4: 지형 기반 이상 궤적 (바퀴 3점 평균 적용)
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

    # 실패 구간 패널티
    if 'ok' in R:
        err_v4[~R['ok']] = 0.5

    MSD = np.nanmean(err_v4**2)
    if MSD < 1e-20:
        MSD = 1e-20

    M = {}
    M['SN_dB'] = -10 * np.log10(MSD)
    M['err_std'] = np.sqrt(MSD)

    # v2 호환 (movmean 기반)
    win_v2 = max(3, min(40, N // 4))
    y_ideal_v2 = np.convolve(R['y0'], np.ones(win_v2) / win_v2, mode='same')
    err_v2 = R['y0'] - y_ideal_v2
    mse_v2 = np.nanmean(err_v2**2)
    if mse_v2 < 1e-20:
        mse_v2 = 1e-20
    M['SN_dB_v2'] = -10 * np.log10(mse_v2)

    # 기타 지표
    M['h_std'] = np.nanstd(R['y0'])
    M['r_range_deg'] = np.rad2deg(np.nanmax(R['ar']) - np.nanmin(R['ar']))
    M['b_range_deg'] = np.rad2deg(np.nanmax(R['bb']) - np.nanmin(R['bb']))

    # 시각화/외부 참조용 필드
    if 'ok' in R:
        valid_mask = R['ok']
    else:
        valid_mask = ~np.isnan(R['y0'])

    M['y0_valid'] = R['y0'][valid_mask]
    M['y_ideal'] = y_ideal_v4[valid_mask]

    return M
