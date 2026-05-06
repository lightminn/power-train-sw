function M = calc_metrics(x_arr, x_t, y_t, R, p)
% calc_metrics  주행 품질 지표 계산  [v4]
%
% ══ v4 추가 수정 (사용자 제안 적극 수용) ════════════════════════
%
%  [M-1] 이상 궤적(y_ideal) 물리적 재정의 ★신규 반영★
%      - 기존: y_ideal = y_terrain(x_arr) + R_w
%              (차체 중심 P0가 지형을 그대로 따라가야 한다고 가정하는 오류)
%      - 수정: 3개 바퀴 접촉점의 평균 높이 기반으로 재정의
%              y_ideal = (y_wr + y_wm + y_wf)/3 + R_w + h_offset
%              (Rocker-Bogie 메커니즘의 본질적인 서스펜션 거동을 올바르게 평가)
%
% ══ v3 수정 내역 ════════════════════════════════════════════════
%
%  [FIX-2] S/N비 정의 정렬
%      - 기존: -10*log10(mean(err²))  (절대 높이 오프셋 포함)
%      - 수정: var(err_demeaned) 기반 (DC 오프셋 제거 후 순수 변동성)
%
%  [호환] M.SN_dB_v2 (movmean 기반) 병행 출력
%
%  [R-1] valid_mask 마스킹 수정
%      - 기존: ~isnan(R.y0) → fill_nan 보간 후 효과 없음
%      - 수정: R.ok (fsolve 수렴 성공 포인트만) 사용
%
% ══ 출력 필드 ════════════════════════════════════════════════════
%   M.SN_dB      : v4 S/N비 [dB] (바퀴 3점 지형 평균, var 기반)
%   M.SN_dB_v2   : v2 S/N비 [dB] (movmean 기반, 하위 호환)
%   M.err_std    : 추종 오차 표준편차 [m]
%   M.h_std      : P0 높이 표준편차 [m]
%   M.r_range_deg: Rocker 각도 범위 [deg]
%   M.b_range_deg: Bogie 각도 범위 [deg]

    N = length(x_arr);
    % [수정] 조용히 평가를 망가뜨리는 하드코딩 디폴트값 제거 및 명시적 에러 처리
    if ~isfield(p, 'R_w')
        error('calc_metrics: 파라미터 구조체 p에 R_w 값이 누락되었습니다.');
    end


    %% ── v4: 지형 기반 이상 궤적 (바퀴 3점 평균 적용) [M-1] ──────
    y_wr = interp1(x_t, y_t, R.xwr, 'linear', 'extrap');
    y_wm = interp1(x_t, y_t, R.xwm, 'linear', 'extrap');
    y_wf = interp1(x_t, y_t, R.xwf, 'linear', 'extrap');
    
    y_avg = (y_wr + y_wm + y_wf) / 3; % 팽창 지형이므로 R_w 더하지 않음

    % [수정] 평지에서의 오프셋을 구조의 '기준 자세(Reference)'로 삼음
    % [수정 1] 해석기 초기 과도 응답을 제외하기 위해 0.1m ~ 0.6m 구간을 평지 기준으로 사용
    idx_flat = (x_arr >= (x_arr(1) + 0.1)) & (x_arr <= (x_arr(1) + 0.6));
    
    if any(idx_flat)
        h_offset_ref = mean(R.y0(idx_flat) - y_avg(idx_flat), 'omitnan');
    else
        h_offset_ref = R.y0(1) - y_avg(1); 
    end
    
    y_ideal_v4 = y_avg + h_offset_ref;
    err_v4 = R.y0 - y_ideal_v4;
    
    
    % [수정] err_demeaned(분산만 평가) 대신 err_v4 자체의 제곱평균(MSD)을 구하여
    % 흔들림(Variance)과 차체 높이 이탈(Bias)을 동시에 강력하게 평가!

    % [수정 2] 실패 구간(NaN) 역설 방어 로직
    % 실패한 지점(R.ok == false)에는 매우 큰 오차(예: 0.5m)를 강제로 부여하여 
    % 실패율이 높은 설계가 S/N비에서 높은 점수를 받는 것을 원천 차단합니다.
    if isfield(R, 'ok')
        err_v4(~R.ok) = 0.5; % 50cm 오차에 해당하는 패널티 부여
    end
    
    MSD = mean(err_v4.^2, 'omitnan');
    if MSD < 1e-20, MSD = 1e-20; end
    M.SN_dB   = -10 * log10(MSD);
    M.err_std = sqrt(MSD);

    %% ── v2 호환 (movmean 기반) ───────────────────────────────────
    win_v2     = max(3, min(40, floor(N/4)));
    y_ideal_v2 = movmean(R.y0, win_v2);
    err_v2     = R.y0 - y_ideal_v2;
    mse_v2     = mean(err_v2.^2, 'omitnan');
    if mse_v2 < 1e-20, mse_v2 = 1e-20; end
    M.SN_dB_v2 = -10 * log10(mse_v2);

    %% ── 기타 지표 ───────────────────────────────────────────────
    M.h_std       = std(R.y0, 'omitnan');
    M.r_range_deg = rad2deg(max(R.ar) - min(R.ar));
    M.b_range_deg = rad2deg(max(R.bb) - min(R.bb));

    %% ── 시각화·외부 참조용 필드 (JointOptSearch Section 11 호환) ─
    if isfield(R, 'ok')
        valid_mask = R.ok;
    else
        valid_mask = ~isnan(R.y0);
    end
    M.y0_valid   = R.y0(valid_mask);
    M.y_ideal    = y_ideal_v4(valid_mask);
end