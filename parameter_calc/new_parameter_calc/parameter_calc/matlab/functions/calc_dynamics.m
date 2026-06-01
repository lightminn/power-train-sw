function D = calc_dynamics(R, x_arr, x_t, y_t, p)
% calc_dynamics  동역학 계산 — 전 모드 통합판  [v4]
%
% ══ gear 5:1 수정 ════════════════════════════════════════════════
%
%  [gear5:1-A] gear_ratio 기본값 30 → 5
%      - p.gear_ratio 미전달 시 fallback 기본값 수정
%
%  [gear5:1-B] F_drv /2 복원  ★물리 모델 수정★
%      - v3-BUG-2에서 /2 제거 → 오류 판명 → 복원
%      - 근거: 2D 모델은 W=mass*g (전체 30kg) 기준
%              실제 6바퀴 분담 → 한쪽 3바퀴 = /2
%              /2 제거 시 tau_motor 2배 과대추정
%
% ══ v4 추가 수정 ══════════════════════════════════════════════════
%
%  [D-4] Savitzky-Golay 필터 도입 (sgolayfilt) ★신규 반영★
%      - 기존: gradient 이중 적용 후 movmean 평활화 → 노이즈 증폭
%      - 수정: sgolayfilt를 이용해 위치 데이터를 먼저 평활화한 후,
%              미분 과정에서도 필터를 적용하여 부드러운 가속도(ay, alpha) 추출
%
%  [D-1] Bogie frame bogie_arm_h sin/cos 오류  ★치명★
%      - 기존: c_h = (1-j_b)*T_b*|cos(bb)| + S_b1*|sin(th_b1)|
%              → wpos 기구학과 불일치 (th_b1=38.5°: 26% 오차)
%      - 수정: c_h = |(1-j_b)*T_b*cos(bb) + S_b1*sin(th_b1+bb)|
%              d_h = |j_b*T_b*cos(bb) + S_b2*sin(th_b2-bb)|
%
%  [D-2] Bogie frame bogie_cb0/db0 bb=0 가정 오류
%      - 기존: cb = (1-j_b)*T_b + S_b1*sin(th_b1)  (bb=0 가정)
%      - 수정: cb = S_b1*cos(th_b1)  (bb≈-π/2 기준, I_bogie 정확도 향상)
%
%  [D-3] v_app 물리 한계 클램프
%      - 기존: max(|vy_cg|, 0.01) → 수치 미분 불연속점에서 수백 m/s 발생
%      - 수정: min(|vy_cg|, v_robot) → 수직 충돌속도 ≤ 수평 이동속도
%
% ══ v3 수정 내역 (v2 대비) ══════════════════════════════════════
%
%  [FIX-1] ay_cg 이중미분 공식 오류 수정  ★치명★
%      - 기존: gradient(gradient(ycg,x_arr)*v) ./ (dt+eps)  → 단위 불일치
%      - 수정: gradient(gradient(ycg,x_arr), x_arr) * v²   → d²y/dt² 정확
%      - movmean 창: 5 → max(11, N/20) 으로 확대 (계단 충격 스파이크 억제)
%
%  [FIX-2] ratio_rb 분모 부호 보존 (calc_stability와 통일)
%      - 기존: a_eff / (b_eff + sign(b_eff)*1e-6)  → b_eff=0이면 /0
%      - 1차: max(b_eff, 1e-3) → b_eff<0을 양수 강제 → 들림 숨김 (폐기)
%      - 최종: |b_eff|<1e-3일 때만 방어, 부호 보존 → 들림 올바르게 감지
%
%  [FIX-3] 슬립비 분모 하한 강화
%      - 기존: p.mu*Nr + 1e-6  → Nr=0이면 slip=수천 → traction_ok 과다 실패
%      - 수정: p.mu*max(Nr, 0.5)  (0.5N 바닥)
%
%  [FIX-4] 관성력 부호 수정  ★치명★
%      - 기존: W_eff = W·cos θ − F_in_y  → 위로 가속 시 법선력 감소 (물리 역전)
%      - 수정: W_eff = W·cos θ + F_in_y  → 위로 가속 시 지면 반력 증가 (올바름)
%      - 근거: ay_cg>0(위 가속) → 관성 반력이 아래로 → 지면을 더 세게 누름
%
%  [FIX-5] 바퀴 회전관성 중복 합산 수정
%      - 기존: tau_in이 3개 모터 모두에 동일하게 더해짐 → 3배 과대
%      - 수정: tau_in을 N 비율로 배분 (각 바퀴 부하에 비례)
%
%  [FIX-6] Rocker 링크 관성 → 3바퀴 N 비율 배분
%      - 기존: Rocker 관성 전체를 뒷바퀴(Wr)에만 귀속
%      - 수정: N_tot 비율로 3바퀴에 배분 (P0 피벗 반력이 3바퀴로 전달)
%
% ══ v2 수정 내역 (v1 대비) ══════════════════════════════════════
%  [1] Lagrangian 링크 관성 항 추가
%  [2] 법선력 음수 클리핑 → 명시적 경고 + liftoff 플래그
%  [3] 출력 필드 추가: tau_rocker_inertia, tau_bogie_inertia, alpha_rocker,
%      alpha_bogie, Nr_raw, Nf_raw, liftoff_r, liftoff_f
%
% ══ 파라미터 ═════════════════════════════════════════════════════
%   rocker_mode: 'linear' / 'triangle' / 'frame'
%   bogie_mode : 'linear' / 'triangle' / 'frame'
%   p.m_rocker_link : Rocker 링크 질량 [kg]  (기본 0.8)
%   p.m_bogie_link  : Bogie  링크 질량 [kg]  (기본 0.5)
%   p.I_rocker_add  : 외부 지정 Rocker 관성 모멘트 [kg·m²]  (기본 0)
%   p.I_bogie_add   : 외부 지정 Bogie  관성 모멘트 [kg·m²]  (기본 0)


    % [신규 추가] filtfilt 함수의 불필요한 경고 메시지 끄기
    warning('off', 'signal:filtfilt:ParseTransferFunction');
    
    %% ── 기본값 ──────────────────────────────────────────────────
    if ~isfield(p,'mu'),              p.mu              = 0.70; end
    if ~isfield(p,'gear_ratio'),      p.gear_ratio      = 5;    end   % ★ gear 5:1
    if ~isfield(p,'eta_gear'),        p.eta_gear        = 0.85; end
    if ~isfield(p,'motor_tau_peak'),  p.motor_tau_peak  = 4.0;  end
    if ~isfield(p,'m_wheel'),         p.m_wheel         = 1.2;  end
    if ~isfield(p,'m_rocker_link'),   p.m_rocker_link   = 0.8;  end
    if ~isfield(p,'m_bogie_link'),    p.m_bogie_link    = 0.5;  end
    if ~isfield(p,'I_rocker_add'),    p.I_rocker_add    = 0;    end
    if ~isfield(p,'I_bogie_add'),     p.I_bogie_add     = 0;    end
    if ~isfield(p,'e_restitution'),   p.e_restitution   = 0.3;  end
    if ~isfield(p,'v_robot'),         p.v_robot         = 1.0;  end
    if ~isfield(p,'step_thresh'),     p.step_thresh     = 5.0;  end
    if ~isfield(p,'CG_offset'),       p.CG_offset       = 0;    end
    if ~isfield(p,'h_CG'),            p.h_CG            = p.h_body*0.5; end
    if ~isfield(p,'rocker_mode'),     p.rocker_mode     = 'linear'; end
    if ~isfield(p,'bogie_mode'),      p.bogie_mode      = 'linear'; end

    N  = length(x_arr);
    W  = p.mass * p.g;

    %% ── ① 바퀴 회전 관성 ────────────────────────────────────────
    I_wheel = 0.5 * p.m_wheel * p.R_w^2;

    %% ── ② Rocker 링크 관성 모멘트 ──────────────────────────────
    [L_r1_eff, L_r2_eff] = rocker_arm_len(p);
    L_r_tot = max(L_r1_eff + L_r2_eff, 1e-4);
    m_r1 = p.m_rocker_link * L_r1_eff / L_r_tot;
    m_r2 = p.m_rocker_link * L_r2_eff / L_r_tot;
    I_rocker = (1/3)*m_r1*L_r1_eff^2 + (1/3)*m_r2*L_r2_eff^2 + p.I_rocker_add;

    %% ── ③ Bogie 링크 관성 모멘트 ───────────────────────────────
    cb0 = bogie_cb0(p);
    db0 = bogie_db0(p);
    I_bogie_wheel = p.m_wheel*cb0^2 + p.m_wheel*db0^2;
    I_bogie_link  = (1/12)*p.m_bogie_link*(cb0+db0)^2 + p.I_bogie_add;
    I_bogie       = I_bogie_wheel + I_bogie_link;
    m_bogie_eff   = I_bogie / (cb0^2 + 1e-9);

%% ── ④ CG 가속도 [filtfilt 0-위상 필터 적용] ──────────────────────
    if isfield(R,'ycg') && ~all(isnan(R.ycg)), ycg = R.ycg; else, ycg = R.y0 + p.h_body * 0.5; end


    % [수정] 이동평균 창 크기를 대폭 축소하여 순간적인 피크 토크(충격)를 둔화시키지 않고 포착    
    sm_win_cg = max(5, floor(N/50));
    kernel_cg = ones(1, sm_win_cg) / sm_win_cg;
    
% [수정] filtfilt를 적용하여 위상 지연(Phase delay) 없이 신호 평활화 달성
    ycg_smooth = filtfilt(kernel_cg, 1, ycg);
    vy_cg      = gradient(ycg_smooth, x_arr) * p.v_robot;
    ay_cg_raw  = gradient(vy_cg, x_arr) * p.v_robot;
    ay_cg      = filtfilt(kernel_cg, 1, ay_cg_raw);
    ay_cg      = min(max(ay_cg, -3 * p.g), 3 * p.g); % 수직 가속도 클램프

    %% ── ⑤ 링크 각가속도 [D-4] S-G 필터 적용 ────────────────────────────
    % [수정] 각가속도 평활화 창 역시 응답 지연(Lag)을 최소화
    sm_win = max(5, floor(N/40));
    kernel = ones(1, sm_win) / sm_win;

    ar_smooth        = filtfilt(kernel, 1, R.ar);
    dar_dt           = gradient(ar_smooth, x_arr) * p.v_robot;
    alpha_rocker_raw = gradient(dar_dt, x_arr) * p.v_robot;
    alpha_rocker     = filtfilt(kernel, 1, alpha_rocker_raw);
    alpha_rocker     = min(max(alpha_rocker, -50), 50);

    bb_smooth        = filtfilt(kernel, 1, R.bb);
    dbb_dt           = gradient(bb_smooth, x_arr) * p.v_robot;
    alpha_bogie_raw  = gradient(dbb_dt, x_arr) * p.v_robot;
    alpha_bogie      = filtfilt(kernel, 1, alpha_bogie_raw);
    alpha_bogie      = min(max(alpha_bogie, -50), 50);

    %% ── ⑥ 계단 에지 감지 ───────────────────────────────────────
    theta_arr = zeros(1,N);
    for i = 1:N
        hf_ = interp1(x_t, y_t, R.xwf(i), 'linear','extrap');
        hr_ = interp1(x_t, y_t, R.xwr(i), 'linear','extrap');
        sp  = R.xwf(i) - R.xwr(i);
        if abs(sp) > 1e-6
            theta_arr(i) = atan2(hf_-hr_, sp);
        end
    end
    dth_dx    = abs(gradient(theta_arr, x_arr));
    edge_mask = dth_dx > deg2rad(p.step_thresh)/0.30;

    %% ── 출력 초기화 ─────────────────────────────────────────────
    D.Nr=zeros(1,N); D.Nm=zeros(1,N); D.Nf=zeros(1,N);
    D.Nr_raw=zeros(1,N); D.Nf_raw=zeros(1,N);
    D.liftoff_r=false(1,N); D.liftoff_f=false(1,N);
    D.Fdr=zeros(1,N); D.Fdm=zeros(1,N); D.Fdf=zeros(1,N);
    D.tau_wheel_r=zeros(1,N); D.tau_wheel_m=zeros(1,N); D.tau_wheel_f=zeros(1,N);
    D.tau_motor_r=zeros(1,N); D.tau_motor_m=zeros(1,N); D.tau_motor_f=zeros(1,N);
    D.tau_inertia=zeros(1,N);
    D.tau_rocker_inertia=zeros(1,N); D.tau_bogie_inertia=zeros(1,N);
    D.alpha_rocker=alpha_rocker; D.alpha_bogie=alpha_bogie;
    D.tau_impact=zeros(1,N); D.tau_impact2=zeros(1,N);
    D.slip_r=zeros(1,N); D.slip_m=zeros(1,N); D.slip_f=zeros(1,N);
    D.power_total=zeros(1,N); D.traction_ok=true(1,N);
    D.theta_local=theta_arr; D.edge_mask=edge_mask;
    D.F_inertia=zeros(1,N); D.F_impact=zeros(1,N); D.F_impact2=zeros(1,N);
    D.ay_cg=ay_cg;

    n_liftoff_r = 0;
    n_liftoff_f = 0;

    %% ── 메인 루프 ────────────────────────────────────────────────
    for i = 1:N
        ar = R.ar(i); bb = R.bb(i); theta = theta_arr(i);

        %% ─ 관성력 ────────────────────────────────────────────────
        % [BUG-FIX] 부호 수정: ay_cg = d²y_CG/dt²  (위 방향 양수)
        %   로봇이 위로 가속(ay>0) → 지면이 더 세게 밀어야 → 법선력 증가
        %   → W_eff_raw = W*cos(θ) + F_in_y + F_imp  (+부호)
        %   기존(- F_in_y)는 위로 가속 시 법선력 감소로 처리해 물리 역전됨
        F_in_y         = p.mass * ay_cg(i);
        D.F_inertia(i) = F_in_y;

        %% ─ 2단계 연쇄 충격 ──────────────────────────────────────
        if edge_mask(i)
            % [D-3] v_app 물리 한계 클램프:
            %   gradient 수치 미분이 격자 불연속점에서 수백 m/s를 만들 수 있음
            %   실제 충돌 수직 접근속도는 수평 이동속도(v_robot)를 초과 불가
            v_app  = min(max(abs(vy_cg(i)), 0.01), p.v_robot);
            dv     = (1 + p.e_restitution) * v_app;
            % [BUG-5] Hertz 접촉 이론으로 t_c 수정
            %   기존: 2*R_w/v_robot = 0.5s (실제 접촉시간의 수백배)
            %   수정: pi*sqrt(m_eff/k_contact), k=1e5 N/m (우레탄 200mm 타이어)
            m_eff_c   = p.m_wheel + m_bogie_eff;
            k_contact = 1e5;
            t_c       = pi * sqrt(m_eff_c / k_contact);
            t_c       = max(t_c, 0.002);   % 최소 2ms
            F_imp1 = (p.m_wheel + m_bogie_eff) * dv / (t_c + 1e-9);
            a_bog  = F_imp1 * cb0 / (I_bogie + 1e-9);
            t_del  = sqrt(pi / max(a_bog, 0.1));
            F_imp2 = F_imp1 * p.e_restitution;
            i2     = min(i + round(t_del*p.v_robot/mean(diff(x_arr))), N);
            D.F_impact(i)   = F_imp1;
            D.F_impact2(i2) = D.F_impact2(i2) + F_imp2;
        end
        F_imp_tot = D.F_impact(i) + D.F_impact2(i);

        %% ─ 유효 중력 성분 ────────────────────────────────────────
        % [BUG-FIX] - F_in_y → + F_in_y
        W_eff_raw = W*cos(theta) + F_in_y + F_imp_tot;
        W_eff_raw = min(W_eff_raw, 5 * W);  % [D-5] 5g 상한 클램프
        
        %% ─ 법선력 분배 ───────────────────────────────────────────
        [c_h, d_h] = bogie_arm_h(bb, p);
        [a_h, b_h] = rocker_arm_h(ar, p);

        a_eff = a_h + p.CG_offset*cos(ar);
        b_eff = b_h - p.CG_offset*cos(ar);

        ratio_fm = d_h / max(c_h, 1e-3);
        % [BUG-FIX] 부호 보존: b_eff<0(CG가 뒷축 후방)이면 Nr_raw<0 → 들림 감지
        %   max(b_eff, 1e-3)은 음수를 양수로 강제 → 들림 숨김 (calc_stability와 불일치)
        %   수정: 절댓값이 1e-3 미만일 때만 방어, 부호는 보존 (calc_stability FIX-G와 통일)
        if abs(b_eff) < 1e-3
            b_eff_safe = sign(b_eff + 1e-9) * 1e-3;
        else
            b_eff_safe = b_eff;
        end
        ratio_rb = a_eff / b_eff_safe;

        Nb_raw = W_eff_raw / (ratio_rb + 1);
        Nr_raw = Nb_raw * ratio_rb;
        Nm_raw = Nb_raw / (1 + ratio_fm);
        Nf_raw = Nm_raw * ratio_fm;

        D.Nr_raw(i) = Nr_raw;
        D.Nf_raw(i) = Nf_raw;

        %% ─ 들림 감지 ────────────────────────────────────────────
        if Nr_raw < 0
            D.liftoff_r(i) = true;
            n_liftoff_r    = n_liftoff_r + 1;
        end
        if Nf_raw < 0
            D.liftoff_f(i) = true;
            n_liftoff_f    = n_liftoff_f + 1;
        end

        Nr = max(Nr_raw, 0);
        Nm = max(Nm_raw, 0);
        Nf = max(Nf_raw, 0);
        D.Nr(i)=Nr; D.Nm(i)=Nm; D.Nf(i)=Nf;

        %% ─ 구동력 ────────────────────────────────────────────────
        % 2D 모델은 '한쪽 면(사이드)' 3바퀴를 시뮬레이션함
        % W=mass*g는 전체 30kg 기준이므로, 저항도 전체 기준
        % 실제 6바퀴(양쪽)가 분담하므로 한쪽 3바퀴는 /2
        % [BUG-2 재검토] /2 복원:
        %   v3에서 BUG-2라고 /2를 제거했으나, 이는 오류임
        %   W = 30kg 전체 중량 사용 → 저항 전체 → 한쪽 3바퀴 분담 = /2
        %   /2 제거 시 tau_motor가 2배 과대 추정됨
        % 오르막(구동)이든 내리막(제동)이든 모터는 토크를 발휘해야 하며,
        % 지면과의 마찰력이 필요하므로 절대값(abs)을 취합니다.
        F_req_total = W * sin(theta) + 0.02 * W * cos(theta);
        F_drv   = abs(F_req_total) / 2;   % 2D 모델이므로 한쪽 면(3바퀴) 분담 -> /2 적용
        
        N_tot = Nr + Nm + Nf;
        if N_tot > 1e-6
            Fdr=F_drv*(Nr/N_tot); Fdm=F_drv*(Nm/N_tot); Fdf=F_drv*(Nf/N_tot);
        else
            Fdr=0; Fdm=0; Fdf=0;
        end
        D.Fdr(i)=Fdr; D.Fdm(i)=Fdm; D.Fdf(i)=Fdf;

        %% ─ 바퀴 토크 ─────────────────────────────────────────────
        tau_wr=Fdr*p.R_w; tau_wm=Fdm*p.R_w; tau_wf=Fdf*p.R_w;
        D.tau_wheel_r(i)=tau_wr; D.tau_wheel_m(i)=tau_wm; D.tau_wheel_f(i)=tau_wf;

        %% ─ 바퀴 회전관성 [FIX-5: N 비율 배분] ──────────────────
        % [BUG-FIX] 기존: abs(ay_cg*cos(theta))/R_w — cos(theta) 이중 보정
        %   ay_cg는 이미 글로벌 수직 가속도이므로 바퀴 접선가속도 = ay_cg/R_w
        tau_in_total = I_wheel * abs(ay_cg(i)) / p.R_w;
        D.tau_inertia(i) = tau_in_total;
        if N_tot > 1e-6
            tau_in_r = tau_in_total * (Nr / N_tot);
            tau_in_m = tau_in_total * (Nm / N_tot);
            tau_in_f = tau_in_total * (Nf / N_tot);
        else
            tau_in_r = tau_in_total / 3;
            tau_in_m = tau_in_total / 3;
            tau_in_f = tau_in_total / 3;
        end

        %% ─ Rocker 링크 관성 [FIX-6: N 비율로 3바퀴 배분] ───────
        tau_rocker_lnk = I_rocker * abs(alpha_rocker(i));
        D.tau_rocker_inertia(i) = tau_rocker_lnk / (p.gear_ratio * p.eta_gear);
        if N_tot > 1e-6
            tau_rk_r = D.tau_rocker_inertia(i) * (Nr / N_tot);
            tau_rk_m = D.tau_rocker_inertia(i) * (Nm / N_tot);
            tau_rk_f = D.tau_rocker_inertia(i) * (Nf / N_tot);
        else
            tau_rk_r = D.tau_rocker_inertia(i) / 3;
            tau_rk_m = D.tau_rocker_inertia(i) / 3;
            tau_rk_f = D.tau_rocker_inertia(i) / 3;
        end

        %% ─ Bogie 링크 관성 ──────────────────────────────────────
        tau_bogie_lnk = I_bogie * abs(alpha_bogie(i));
        D.tau_bogie_inertia(i) = tau_bogie_lnk / (p.gear_ratio * p.eta_gear);
        N_bogie = Nm + Nf;
        if N_bogie > 1e-6
            tau_bg_m = D.tau_bogie_inertia(i) * (Nm / N_bogie);
            tau_bg_f = D.tau_bogie_inertia(i) * (Nf / N_bogie);
        else
            tau_bg_m = D.tau_bogie_inertia(i) / 2;
            tau_bg_f = D.tau_bogie_inertia(i) / 2;
        end

        %% ─ 충격 토크 ─────────────────────────────────────────────
        ti1 = D.F_impact(i)  * p.R_w;
        ti2 = D.F_impact2(i) * p.R_w;
        D.tau_impact(i)=ti1; D.tau_impact2(i)=ti2;

        %% ─ 모터 토크 합산 ───────────────────────────────────────
        ge = p.gear_ratio * p.eta_gear;
        D.tau_motor_r(i) = (tau_wr + tau_in_r + tau_rk_r)                     / ge;
        D.tau_motor_m(i) = (tau_wm + tau_in_m + tau_rk_m + tau_bg_m + ti2)    / ge;
        D.tau_motor_f(i) = (tau_wf + tau_in_f + tau_rk_f + tau_bg_f + ti1)    / ge;

        %% ─ 마찰 여유도 (Friction Utilization) ────────────────────────
        MIN_N = 0.5;
        % 물리적 명칭 정정: 요구 마찰력 / 가용 마찰력 (1.0 이상이면 슬립 발생)
        D.slip_r(i) = abs(Fdr) / (p.mu * max(Nr, MIN_N));
        D.slip_m(i) = abs(Fdm) / (p.mu * max(Nm, MIN_N));
        D.slip_f(i) = abs(Fdf) / (p.mu * max(Nf, MIN_N));

        %% ─ 견인력 판정 ───────────────────────────────────────────
        motor_ok   = max([D.tau_motor_r(i), D.tau_motor_m(i), D.tau_motor_f(i)]) ...
                     <= p.motor_tau_peak;
        slip_ok    = max([D.slip_r(i), D.slip_m(i), D.slip_f(i)]) < 1.0;
        no_liftoff = ~D.liftoff_r(i) && ~D.liftoff_f(i);
        D.traction_ok(i) = slip_ok && motor_ok && no_liftoff;

        %% ─ 소비 전력 ─────────────────────────────────────────────
        % tau_motor_* 는 한쪽 면(3바퀴) 기준 → 양쪽 면 반영 위해 × 2
        om = (p.v_robot / p.R_w) * p.gear_ratio;
        D.power_total(i) = (D.tau_motor_r(i)+D.tau_motor_m(i)+D.tau_motor_f(i)) * om * 2;
    end

    %% ── 들림 요약 출력 ──────────────────────────────────────────
    if n_liftoff_r > 0 || n_liftoff_f > 0
        fprintf('  [⚠️  법선력 경고] 뒷바퀴 들림: %d pts / 앞바퀴 들림: %d pts\n', ...
                n_liftoff_r, n_liftoff_f);
        fprintf('     Nr_raw 범위: [%.1f, %.1f] N  |  Nf_raw 범위: [%.1f, %.1f] N\n', ...
                min(D.Nr_raw), max(D.Nr_raw), min(D.Nf_raw), max(D.Nf_raw));
        fprintf('     → 구조 재검토 또는 calc_stability()로 ZMP 분석 권장\n');
    end

    % [신규 추가] 기구학 실패 포인트의 가짜 토크/슬립 데이터를 명시적으로 파기 (NaN 처리)
    % 이를 통해 plot_sim 그래프나 이후 통계 산출 시 오염된 데이터가 쓰이는 것을 막음
    if isfield(R, 'ok')
        invalid = ~R.ok;
        D.tau_motor_r(invalid) = NaN;
        D.tau_motor_m(invalid) = NaN;
        D.tau_motor_f(invalid) = NaN;
        D.slip_r(invalid)      = NaN;
        D.slip_m(invalid)      = NaN;
        D.slip_f(invalid)      = NaN;
    end

    %% ── 계단 피크 토크 ──────────────────────────────────────────
    D.tau_max_arr = max([D.tau_motor_r; D.tau_motor_m; D.tau_motor_f]);
    stair_zone    = conv(double(edge_mask), ones(1,11)/11, 'same') > 0;
    
    % [신규 추가] 기구학 실패 포인트가 통계에 정답처럼 포함되는 것을 마스킹
    valid_mask = R.ok; 
    valid_stair = stair_zone & valid_mask;
    
    if any(valid_stair)
        D.stair_torque_peak = prctile(D.tau_max_arr(valid_stair), 95);
    elseif any(valid_mask)
        D.stair_torque_peak = prctile(D.tau_max_arr(valid_mask), 95);
    else
        D.stair_torque_peak = 100; % 전 구간 실패 시 강력한 페널티
    end
    D.stair_torque_max = max(D.tau_max_arr(valid_mask));

    %% ── 링크 관성 기여 요약 통계 ────────────────────────────────
    D.tau_link_max   = max(D.tau_rocker_inertia + D.tau_bogie_inertia);
    D.tau_link_ratio = D.tau_link_max / (D.stair_torque_max + 1e-9);

end

%% ════════════════════════════════════════════════════════════
%% 헬퍼: Rocker 암 실제 길이 (관성 모멘트 계산용)
%% ════════════════════════════════════════════════════════════
function [L1, L2] = rocker_arm_len(p)
    switch lower(p.rocker_mode)
        case 'linear'
            if isfield(p,'a_r'), L1=p.a_r; else, L1=0.22; end
            if isfield(p,'b_r'), L2=p.b_r; else, L2=0.28; end
        case 'triangle'
            L1 = p.L_r1;  L2 = p.L_r2;
        case 'frame'
            L1 = sqrt(((1-p.j_r)*p.T_r)^2 + p.S_r1^2);
            L2 = sqrt((   p.j_r *p.T_r)^2 + p.S_r2^2);
        otherwise
            if isfield(p,'a_r'), L1=p.a_r; else, L1=0.22; end
            if isfield(p,'b_r'), L2=p.b_r; else, L2=0.28; end
    end
    L1 = max(L1, 0.05);  L2 = max(L2, 0.05);
end

%% ════════════════════════════════════════════════════════════
%% 헬퍼: Rocker 수평 투영 팔길이  (ar 각도 반영)
%% ════════════════════════════════════════════════════════════
function [a_h, b_h] = rocker_arm_h(ar, p)
    switch lower(p.rocker_mode)
        case 'linear'
            a_h = p.a_r * abs(cos(ar));
            b_h = p.b_r * abs(cos(ar));
        case 'triangle'
           % wpos.m 실제 코드 기준: ang_pb = ar - alpha_r/2
            ang_pb = ar - p.alpha_r/2;
            ang_wr = ar - pi + p.alpha_r/2;
            a_h = max(p.L_r1 * abs(cos(ang_pb)), 0.01);
            b_h = max(p.L_r2 * abs(cos(ang_wr)), 0.01);
        case 'frame'
            % wpos.m frame 규약과 동일:
            %   수평 투영 = T_r 수평 성분 + S_r*sin(th) (수평 성분)
            %   수직 지지대의 수평 투영만 a_h/b_h에 기여
            a_h = max((1-p.j_r)*p.T_r*abs(cos(ar)) + p.S_r1*abs(sin(p.th_r1)), 0.01);
            b_h = max(   p.j_r *p.T_r*abs(cos(ar)) + p.S_r2*abs(sin(p.th_r2)), 0.01);
        otherwise
            if isfield(p,'a_r'), a_h=p.a_r*abs(cos(ar)); else, a_h=0.22; end
            if isfield(p,'b_r'), b_h=p.b_r*abs(cos(ar)); else, b_h=0.28; end
    end
    a_h = max(a_h, 0.01);  b_h = max(b_h, 0.01);
end

%% ════════════════════════════════════════════════════════════
%% 헬퍼: Bogie 수평 투영 팔길이  (bb 각도 반영)
%% ════════════════════════════════════════════════════════════
function [c_h, d_h] = bogie_arm_h(bb, p)
    switch lower(p.bogie_mode)
        case 'linear'
            c_h = max(p.c_b * abs(cos(bb)), 0.01);
            d_h = max(p.d_b * abs(cos(bb)), 0.01);
        case 'triangle'
            % wpos.m 규약: 수직 아래(-π/2+bb) 기준 ±beta_b/2 벌어짐
            ang_vert = -pi/2 + bb;
            ang_wf   = ang_vert + p.beta_b/2;
            ang_wm   = ang_vert - p.beta_b/2;
            c_h = max(p.L_b1 * abs(cos(ang_wf)), 0.01);
            d_h = max(p.L_b2 * abs(cos(ang_wm)), 0.01);
        case 'frame'
            % [D-1] wpos.m 기구학 정확 반영:
            %   Wf = Pb + (1-j_b)*T_b*u_b + S_b1*(sin(th_b1)*u_b - cos(th_b1)*n_b)
            %   수평투영 c_h = |(1-j_b)*T_b*cos(bb) + S_b1*sin(th_b1+bb)|
            %   (bb≈-π/2: c_h≈S_b1*cos(th_b1)  ← 이전 sin(th_b1) 대비 ~26% 오차 수정)
            c_h = max(abs((1-p.j_b)*p.T_b*cos(bb) + p.S_b1*sin(p.th_b1+bb)), 0.01);
            d_h = max(abs(   p.j_b *p.T_b*cos(bb) + p.S_b2*sin(p.th_b2-bb)), 0.01);
        otherwise
            if isfield(p,'c_b'), c_h=max(p.c_b*abs(cos(bb)),0.01); else, c_h=0.14; end
            if isfield(p,'d_b'), d_h=max(p.d_b*abs(cos(bb)),0.01); else, d_h=0.14; end
    end
    c_h = max(c_h, 0.01);  d_h = max(d_h, 0.01);
end

%% ════════════════════════════════════════════════════════════
%% 헬퍼: Bogie 평균 팔길이  (관성 모멘트용, bb=0 근사)
%% ════════════════════════════════════════════════════════════
function cb = bogie_cb0(p)
    switch lower(p.bogie_mode)
        case 'linear'
            if isfield(p,'c_b'), cb=p.c_b; else, cb=0.14; end
        case 'triangle'
            % 수직기준 ±beta_b/2: bb=0 근사 시 수평폭 = L × sin(beta_b/2)
            cb = p.L_b1 * abs(sin(p.beta_b/2));
        case 'frame'
            % [D-2] bb≈-π/2 기준: T_b 수평성분≈0, c_h≈S_b1*cos(th_b1)
            %   이전: (1-j_b)*T_b + S_b1*sin(th_b1) → bb=0 가정 → I_bogie 과대
            cb = p.S_b1 * abs(cos(p.th_b1));
        otherwise
            if isfield(p,'c_b'), cb=p.c_b; else, cb=0.14; end
    end
    cb = max(cb, 0.04);
end

function db = bogie_db0(p)
    switch lower(p.bogie_mode)
        case 'linear'
            if isfield(p,'d_b'), db=p.d_b; else, db=0.14; end
        case 'triangle'
            db = p.L_b2 * abs(sin(p.beta_b/2));
        case 'frame'
            % [D-2] bb≈-π/2 기준: d_h≈S_b2*cos(th_b2)
            db = p.S_b2 * abs(cos(p.th_b2));
        otherwise
            if isfield(p,'d_b'), db=p.d_b; else, db=0.14; end
    end
    db = max(db, 0.04);
end