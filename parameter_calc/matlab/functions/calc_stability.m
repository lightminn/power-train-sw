
% calc_stability  전복 안정성 분석 — ZMP + Tip-over Index (TOI)  [v3]
%
% ══ v3 추가 수정 ════════════════════════════════════════════════
%
%  [S-1] 차체 하단 간섭(Collision) 검사 로직 추가 ★신규 반영★
%      - 지형(y_t)과 차체 하단(R.y0 - h_body/2) 간의 Clearance를 계산하여
%        음수가 될 경우(모서리 걸림) risk_level을 'danger'로 격하합니다.
%
% ══ v2 수정 내역 ════════════════════════════════════════════════
%
%  [FIX-1] ratio_rb 분모 부호 보존 (들림 감지 복원)
%      - 기존: b_eff + sign(b_eff+1e-9)*1e-6  → b_eff<0이면 분모 더 음수 → ratio 역전
%      - 1차 수정안: max(b_eff, 1e-3) → b_eff<0을 양수로 강제 → 들림 숨김 (폐기)
%      - 최종 수정: |b_eff|<1e-3일 때만 방어, 부호는 보존
%              b_eff<0(CG 뒷축 후방) → ratio_rb<0 → Nr_raw<0 → 뒷바퀴 들림 올바르게 감지
%
%  [D-1] bogie_arm_h_local frame 모드 D-1 적용 (calc_dynamics와 통일)
%      - 기존: (1-j_b)*T_b*|cos(bb)| + S_b1*|sin(th_b1)| → bb=0 가정, 26% 오차
%      - 수정: |(1-j_b)*T_b*cos(bb) + S_b1*sin(th_b1+bb)|  (wpos 기구학 정확 반영)
%
%  [FIX-2] ZMP N_total 임계값 강화
%      - 기존: abs(N_total) > 1e-3 (0.001N — 사실상 항상 계산)
%      - 수정: N_total > W*0.05  (중량 5% 이상일 때만 ZMP 유효 계산)
%              이하면 x_cg로 대체 (들림 상태에서 ZMP 발산 방지)
%
%  [FIX-3] risk_level 임계값 정렬 (LIFTOFF_MAX 기준 맞춤)
%      - 기존: danger 판정이 liftoff_ratio > 0.05 — LIFTOFF_MAX=0.02와 불일치
%      - 수정: 인자 liftoff_max 수용, 없으면 기본값 0.02 사용
%              위험 등급이 JointOptSearch 강성 제약과 일관성 유지
%
%  [FIX-4] TOI 방향 명시 (xwf=전방, xwr=후방 확인)
%      - 로봇 전진 방향: xwf > xwr  →  sp_max=xwf(전방), sp_min=xwr(후방)
%      - TOI_front = (sp_max - x_cg)/sp_width  = CG가 전방축(Wf)에서 얼마나 안쪽
%      - TOI_rear  = (x_cg - sp_min)/sp_width  = CG가 후방축(Wr)에서 얼마나 안쪽
%      - TOI > 0.5: CG가 중앙에 가까움,  TOI→0: 해당 바퀴 들림 임박
%
% ══ 입력 ════════════════════════════════════════════════════════
%   R          : kin_sim 출력 구조체
%   x_arr      : Rocker pivot x 좌표 배열 [1×N]
%   x_t, y_t   : 지형 배열
%   p          : 파라미터 구조체
%   (선택) p.liftoff_max : 위험 판정 들림 비율 임계값 (기본 0.02)
%
% ══ 출력 S 필드 ══════════════════════════════════════════════════
%   S.x_zmp, S.x_sp_min/max, S.zmp_margin, S.zmp_ok
%   S.TOI, S.TOI_front, S.TOI_rear
%   S.Nr_raw, S.Nf_raw, S.liftoff_r, S.liftoff_f
%   S.min_TOI, S.min_zmp_margin, S.n_liftoff, S.liftoff_ratio
%   S.clearance, S.min_clearance, S.is_collision
%   S.risk_level : 'safe' / 'warning' / 'danger'

function S = calc_stability(R, x_arr, x_t, y_t_raw, y_t_env, p)
% calc_stability  전복 안정성 분석 — ZMP + Tip-over Index (TOI) + 다점 간섭 검사

    %% ── 기본값 ──────────────────────────────────────────────────
    if ~isfield(p,'CG_offset'),    p.CG_offset   = 0; end
    if ~isfield(p,'liftoff_max'),  p.liftoff_max = 0.02; end
    if ~isfield(p,'phi_r0'),       p.phi_r0      = 0; end
    
    N = length(x_arr);
    W = p.mass * p.g;
    
    %% ── CG 위치 (수평 X 좌표만 사용) ──────────────────────────────
    if isfield(R,'xcg') && ~all(isnan(R.xcg))
        x_cg = R.xcg; 
    else
        x_cg = x_arr + p.CG_offset * cos(R.ar);
    end
    
    %% ── 초기화 ──────────────────────────────────────────────────
    S.x_zmp      = NaN(1,N);
    S.x_sp_min   = NaN(1,N);
    S.x_sp_max   = NaN(1,N);
    S.zmp_margin = NaN(1,N);
    S.zmp_ok     = false(1,N);
    
    S.TOI_front  = NaN(1,N);
    S.TOI_rear   = NaN(1,N);
    S.TOI        = NaN(1,N);
    
    S.Nr_raw     = NaN(1,N);
    S.Nf_raw     = NaN(1,N);
    S.liftoff_r  = false(1,N);
    S.liftoff_f  = false(1,N);
    
    for i = 1:N
        if isfield(R, 'ok') && ~R.ok(i)
            continue; 
        end
        
        xwf = R.xwf(i);  xwm = R.xwm(i);  xwr = R.xwr(i);
        if any(isnan([xwf xwm xwr]))
            continue; 
        end
        
        %% ── ① 법선력 재계산 (팽창된 지형 y_t_env 사용) ────────────────
        sp = xwf - xwr;
        if abs(sp) < 1e-6
            sp = 1e-6; 
        end
        
        hf_ = interp1(x_t, y_t_env, xwf, 'linear', 'extrap');
        hr_ = interp1(x_t, y_t_env, xwr, 'linear', 'extrap');
        theta = atan2(hf_ - hr_, sp);
        
        [a_h, b_h] = rocker_arm_h_local(R.ar(i), p);
        [c_h, d_h] = bogie_arm_h_local(R.bb(i),  p);
        
        W_cos = W * cos(theta);
        a_eff = a_h + p.CG_offset * cos(R.ar(i));
        b_eff = b_h - p.CG_offset * cos(R.ar(i));
        
        ratio_fm = d_h / max(c_h, 1e-3);
        
        if abs(b_eff) < 1e-3
            b_eff_safe = sign(b_eff + 1e-9) * 1e-3;
        else
            b_eff_safe = b_eff;
        end
        
        ratio_rb = a_eff / b_eff_safe;
        
        Nb_raw = W_cos / (ratio_rb + 1);
        Nr_raw = Nb_raw * ratio_rb;
        Nm_raw = Nb_raw / (1 + ratio_fm);
        Nf_raw = Nm_raw * ratio_fm;
        
        S.Nr_raw(i) = Nr_raw;
        S.Nf_raw(i) = Nf_raw;
        S.liftoff_r(i) = Nr_raw < 0;
        S.liftoff_f(i) = Nf_raw < 0;
        
        %% ── ② ZMP 계산 ──────────────────────────────────
        N_total = Nr_raw + Nm_raw + Nf_raw;
        if N_total > W * 0.05
            x_zmp_i = (Nr_raw*xwr + Nm_raw*xwm + Nf_raw*xwf) / N_total;
        else
            x_zmp_i = x_cg(i);
        end
        S.x_zmp(i) = x_zmp_i;
        
        sp_min = min(xwf, xwr);   
        sp_max = max(xwf, xwr);   
        S.x_sp_min(i) = sp_min;
        S.x_sp_max(i) = sp_max;
        
        S.zmp_margin(i) = min(x_zmp_i - sp_min, sp_max - x_zmp_i);
        S.zmp_ok(i) = (x_zmp_i >= sp_min) && (x_zmp_i <= sp_max);
        
        %% ── ③ TOI 계산 ──────────────────────────────────
        sp_width = sp_max - sp_min;
        if sp_width < 0.05
            S.TOI_front(i) = 0.5;
            S.TOI_rear(i)  = 0.5;
            S.TOI(i)       = 0.5;
            continue;
        end
        
        TOI_f = max(min((sp_max - x_cg(i)) / sp_width, 2.0), -2.0);
        TOI_r = max(min((x_cg(i) - sp_min) / sp_width, 2.0), -2.0);
        
        S.TOI_front(i) = TOI_f;
        S.TOI_rear(i)  = TOI_r;
        S.TOI(i)       = min(TOI_f, TOI_r);
    end
    
    %% ── ④ 차체 간섭 (Collision) 검사 [진정한 Line-Segment 다점 스캔으로 고도화] ──
    % 단순 꼭짓점이 아닌, 로봇을 구성하는 '모든 선분(Link)'을 잘게 쪼개어
    % 선분 중간이 뾰족한 지형(계단 모서리 등)에 긁히는 현상까지 완벽히 잡아냅니다.
    
    N_div = 20; % 각 선분을 20개의 점으로 촘촘하게 분할 (해상도 약 1~2cm 수준)
    
    % 최대 생성될 선분의 개수 계산
    n_segs = 0;
    if strcmpi(p.rocker_mode, 'frame'), n_segs = n_segs + 3; else, n_segs = n_segs + 2; end
    if strcmpi(p.bogie_mode, 'frame'),  n_segs = n_segs + 3; else, n_segs = n_segs + 2; end
    
    n_pts = n_segs * N_div; 
    S.clearance = NaN(1, N);
    
    % 0~1 사이의 선형 보간용 비율 벡터 (사전 계산)
    t_div = linspace(0, 1, N_div);
    
    for i = 1:N
        if ~isfield(R, 'ok') || ~R.ok(i)
            continue; 
        end
        
        X = [R.y0(i); R.ar(i); R.bb(i)];
        [Wf, Wm, Wr, Pb, ~] = wpos(X, x_arr(i), p);
        P0 = [x_arr(i); R.y0(i)];
        
        pts_x = zeros(1, n_pts);
        pts_y = zeros(1, n_pts);
        idx = 1;
        
        % --- 1. Rocker 선분(Segment) 추출 및 분할 ---
        if strcmpi(p.rocker_mode, 'frame')
            ar_e = R.ar(i) + p.phi_r0; 
            ur = [cos(ar_e); sin(ar_e)];
            Ptr = P0 - p.j_r * p.T_r * ur;
            Ptf = P0 + (1-p.j_r) * p.T_r * ur;
            
            % ① Ptr -> Ptf (메인 빔)
            pts_x(idx:idx+N_div-1) = Ptr(1) + (Ptf(1)-Ptr(1))*t_div;
            pts_y(idx:idx+N_div-1) = Ptr(2) + (Ptf(2)-Ptr(2))*t_div; idx = idx + N_div;
            % ② Ptf -> Pb (전방 지지대)
            pts_x(idx:idx+N_div-1) = Ptf(1) + (Pb(1)-Ptf(1))*t_div;
            pts_y(idx:idx+N_div-1) = Ptf(2) + (Pb(2)-Ptf(2))*t_div; idx = idx + N_div;
            % ③ Ptr -> Wr (후방 지지대)
            pts_x(idx:idx+N_div-1) = Ptr(1) + (Wr(1)-Ptr(1))*t_div;
            pts_y(idx:idx+N_div-1) = Ptr(2) + (Wr(2)-Ptr(2))*t_div; idx = idx + N_div;
        else
            % Triangle/Linear 모드
            % ① P0 -> Pb
            pts_x(idx:idx+N_div-1) = P0(1) + (Pb(1)-P0(1))*t_div;
            pts_y(idx:idx+N_div-1) = P0(2) + (Pb(2)-P0(2))*t_div; idx = idx + N_div;
            % ② P0 -> Wr
            pts_x(idx:idx+N_div-1) = P0(1) + (Wr(1)-P0(1))*t_div;
            pts_y(idx:idx+N_div-1) = P0(2) + (Wr(2)-P0(2))*t_div; idx = idx + N_div;
        end
        
        % --- 2. Bogie 선분(Segment) 추출 및 분할 ---
        if strcmpi(p.bogie_mode, 'frame')
            ubb = [cos(R.bb(i)); sin(R.bb(i))];
            Pbm = Pb - p.j_b * p.T_b * ubb;
            Pbf = Pb + (1-p.j_b) * p.T_b * ubb;
            
            % ① Pbm -> Pbf (메인 빔)
            pts_x(idx:idx+N_div-1) = Pbm(1) + (Pbf(1)-Pbm(1))*t_div;
            pts_y(idx:idx+N_div-1) = Pbm(2) + (Pbf(2)-Pbm(2))*t_div; idx = idx + N_div;
            % ② Pbf -> Wf (전방 지지대)
            pts_x(idx:idx+N_div-1) = Pbf(1) + (Wf(1)-Pbf(1))*t_div;
            pts_y(idx:idx+N_div-1) = Pbf(2) + (Wf(2)-Pbf(2))*t_div; idx = idx + N_div;
            % ③ Pbm -> Wm (후방 지지대)
            pts_x(idx:idx+N_div-1) = Pbm(1) + (Wm(1)-Pbm(1))*t_div;
            pts_y(idx:idx+N_div-1) = Pbm(2) + (Wm(2)-Pbm(2))*t_div; 
        else
            % Triangle/Linear 모드
            % ① Pb -> Wf
            pts_x(idx:idx+N_div-1) = Pb(1) + (Wf(1)-Pb(1))*t_div;
            pts_y(idx:idx+N_div-1) = Pb(2) + (Wf(2)-Pb(2))*t_div; idx = idx + N_div;
            % ② Pb -> Wm
            pts_x(idx:idx+N_div-1) = Pb(1) + (Wm(1)-Pb(1))*t_div;
            pts_y(idx:idx+N_div-1) = Pb(2) + (Wm(2)-Pb(2))*t_div; 
        end
        
        % 3. 배열에 담긴 100~120개의 선분 궤적 점들에 대해 일괄 지형 높이 조회
        terr_y = interp1(x_t, y_t_raw, pts_x, 'linear', 'extrap');
        
        % 4. 여유 공간(Clearance) 산출: 빔 중간이 걸리면 음수가 됨!
        clearances = pts_y - terr_y;
        S.clearance(i) = min(clearances);
    end
    
    S.min_clearance = min(S.clearance, [], 'omitnan');
    S.is_collision = S.min_clearance < 0.01; % 1cm 이하 마진이면 충돌(Collision) 처리

    %% ── 전체 통계 ───────────────────────────────────────────────
    S.min_TOI        = min(S.TOI,        [], 'omitnan');
    S.min_zmp_margin = min(S.zmp_margin, [], 'omitnan');
    
    S.n_liftoff     = sum(S.liftoff_r | S.liftoff_f);
    S.liftoff_ratio = S.n_liftoff / N;
    
    S.n_zmp_out = sum(~S.zmp_ok); % [수정] 사용되지 않던 변수를 구조체에 저장
    pct_zmpout  = S.n_zmp_out / N;   
    
    %% ── 위험 등급 판정 ──────────────────────────────────────────
    if S.min_TOI < 0 || S.liftoff_ratio > p.liftoff_max || pct_zmpout > 0.50 || S.is_collision
        S.risk_level = 'danger';
    elseif S.min_TOI < 0.15 || S.liftoff_ratio > p.liftoff_max*0.5 || pct_zmpout > 0.20
        S.risk_level = 'warning';
    else
        S.risk_level = 'safe';
    end

    %% ── 콘솔 출력 ───────────────────────────────────────────────
    switch S.risk_level
        case 'safe',    badge = '✅ SAFE   ';
        case 'warning', badge = '⚠️  WARNING';
        case 'danger',  badge = '❌ DANGER ';
    end
    
    col_warn = '';
    if S.is_collision, col_warn = ' (간섭!)'; end
    
    fprintf('  [안정성] %s  TOI_min=%.3f  ZMP이탈=%.1f%%  들림=%.1f%%  여유공간=%.3fm%s\n', ...
            badge, S.min_TOI, pct_zmpout*100, S.liftoff_ratio*100, S.min_clearance, col_warn);
end

%% ════════════════════════════════════════════════════════════
%% 로컬 헬퍼: rocker 수평 팔길이 (calc_dynamics와 동일 로직)
%% ════════════════════════════════════════════════════════════
function [a_h, b_h] = rocker_arm_h_local(ar, p)
    if ~isfield(p,'rocker_mode'), p.rocker_mode = 'linear'; end
    switch lower(p.rocker_mode)
        case 'linear'
            a_h = p.a_r * abs(cos(ar));
            b_h = p.b_r * abs(cos(ar));
        case 'triangle'
            ang_pb = ar - p.alpha_r/2;
            ang_wr = ar - pi + p.alpha_r/2;
            a_h = max(p.L_r1 * abs(cos(ang_pb)), 0.01);
            b_h = max(p.L_r2 * abs(cos(ang_wr)), 0.01);
        case 'frame'
            a_h = max((1-p.j_r)*p.T_r*abs(cos(ar)) + p.S_r1*abs(sin(p.th_r1)), 0.01);
            b_h = max(   p.j_r *p.T_r*abs(cos(ar)) + p.S_r2*abs(sin(p.th_r2)), 0.01);
        otherwise
            if isfield(p,'a_r'), a_h=p.a_r*abs(cos(ar)); else, a_h=0.22; end
            if isfield(p,'b_r'), b_h=p.b_r*abs(cos(ar)); else, b_h=0.28; end
    end
    a_h = max(a_h, 0.01);  b_h = max(b_h, 0.01);
end

function [c_h, d_h] = bogie_arm_h_local(bb, p)
    if ~isfield(p,'bogie_mode'), p.bogie_mode = 'linear'; end
    switch lower(p.bogie_mode)
        case 'linear'
            c_h = max(p.c_b * abs(cos(bb)), 0.01);
            d_h = max(p.d_b * abs(cos(bb)), 0.01);
        case 'triangle'
            ang_vert = -pi/2 + bb;
            ang_wf   = ang_vert + p.beta_b/2;
            ang_wm   = ang_vert - p.beta_b/2;
            c_h = max(p.L_b1 * abs(cos(ang_wf)), 0.01);
            d_h = max(p.L_b2 * abs(cos(ang_wm)), 0.01);
        case 'frame'
            c_h = max(abs((1-p.j_b)*p.T_b*cos(bb) + p.S_b1*sin(p.th_b1+bb)), 0.01);
            d_h = max(abs(   p.j_b *p.T_b*cos(bb) + p.S_b2*sin(p.th_b2-bb)), 0.01);
        otherwise
            if isfield(p,'c_b'), c_h=max(p.c_b*abs(cos(bb)),0.01); else, c_h=0.14; end
            if isfield(p,'d_b'), d_h=max(p.d_b*abs(cos(bb)),0.01); else, d_h=0.14; end
    end
    c_h = max(c_h, 0.01);  d_h = max(d_h, 0.01);
end