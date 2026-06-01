function R = kin_sim(x_arr, x_t, y_t, p)
% kin_sim  통합 역기구학 시뮬레이션  [v4]
%
% ══ v4 추가 수정 ════════════════════════════════════════════════
%
%  [R-2] fsolve 알고리즘 최적화 ★신규 반영★
%      - 'levenberg-marquardt' → 'trust-region-dogleg' 변경
%      - 3x3 정방행렬 자코비안 시스템에서 수렴 속도를 대폭 향상시켜
%        전체 최적화 루프의 최대 병목을 해소합니다.
%
%  [R-3] fill_nan 외삽 방식 변경 ★신규 반영★
%      - 'extrap' 시 'linear' 대신 'nearest' 사용
%      - 수렴 실패 구간이 지형 끝단일 경우, 선형 외삽이 비현실적인 각도로
%        발산하는 현상을 막아 후속 동역학 계산의 붕괴를 방지합니다.
%
% ══ v3 수정 내역 ════════════════════════════════════════════════
%
%  [FIX-1] bogie_mode 필드 인식 추가  ★치명★
%      - 기존: p.bogie_type만 확인 → JointOptSearch가 p.bogie_mode 설정 시
%              항상 'legacy' 분기 → bb0=0 (잘못된 초기값 → 수렴 실패율 급증)
%      - 수정: p.bogie_mode 우선 확인, 없으면 p.bogie_type fallback
%
%  [FIX-2] triangle 모드 bb0 올바른 초기값
%      - 기존: bb0 = -(π/2 - alpha_b/2)  (구 wpos_v2 각도 정의 기반)
%      - 수정: bb0 = beta_b/2             (wpos.m 수직기준 정의 기반)
%              평지에서 ang_wf = (-π/2+bb) - beta_b/2 ≈ -π/2 를 만족
%
%  [FIX-3] 오류 메시지 안전화
%      - frame/triangle 모드에서 p.a_r이 없으므로 오류 메시지가 추가 오류 유발
%
% ══ 출력 필드 ═══════════════════════════════════════════════════
%   R.y0, ar, bb                 : 자세
%   R.xwf/ywf, xwm/ywm, xwr/ywr : 바퀴 위치
%   R.xpb/ypb                    : Bogie 피벗 Pb
%   R.xcg/ycg                    : 차체 CG
%   R.ok                         : 수렴 성공 여부
%   R.fail_idx                   : 실패 포인트 인덱스
%   R.fail_rate                  : 실패 비율 [0~1]

    N = length(x_arr);

    R.y0  = NaN(1,N); R.ar  = NaN(1,N); R.bb  = NaN(1,N);
    R.xwf = NaN(1,N); R.ywf = NaN(1,N);
    R.xwm = NaN(1,N); R.ywm = NaN(1,N);
    R.xwr = NaN(1,N); R.ywr = NaN(1,N);
    R.xpb = NaN(1,N); R.ypb = NaN(1,N);
    R.xcg = NaN(1,N); R.ycg = NaN(1,N);
    R.ok  = true(1,N);

    % [R-2] fsolve 알고리즘 최적화 및 스케일링(TypicalX) 적용
    % y0는 길이 스케일(약 R_w), ar과 bb는 라디안 스케일(약 0.1)임을 명시하여 자코비안 품질 향상
    opts = optimoptions('fsolve', ...
        'Display','off', 'TolFun',1e-10, 'TolX',1e-10, ...
        'MaxIterations',600, 'Algorithm','trust-region-dogleg', ...
        'TypicalX', [p.R_w; 0.1; 0.1]);
    %% ── bb 초기값 결정 [FIX-1, FIX-2] ──────────────────────────
    % bogie_mode 우선, 없으면 bogie_type fallback
    if isfield(p,'bogie_mode')
        bmode = lower(p.bogie_mode);
    elseif isfield(p,'bogie_type')
        bmode = lower(p.bogie_type);
    else
        bmode = 'legacy';
    end

    switch bmode
        case 'triangle'
            % [FIX] wpos.m 규약 확인:
            %   ang_wf = (-π/2 + bb) - beta_b/2
            %   평지 접촉 조건: Wf가 Pb 직하방 → ang_wf = -π/2
            %   → (-π/2 + bb) - beta_b/2 = -π/2
            %   → bb = beta_b/2  ← 이게 맞는 초기값
            % 단, ceq에서 fsolve가 bb를 자유변수로 풀므로 초기값은
            % 평지 접촉 근방이면 충분. bb=beta_b/2가 정확한 평지 초기값.
            if isfield(p,'beta_b')
                bb0 = p.beta_b / 2;
            else
                bb0 = deg2rad(45);
            end
        case {'frame', 'trapezoid'}
            bb0 = 0; % Frame 빔이 수평으로 시작하도록 0으로 수정
        otherwise
            bb0 = 0;
    end

    X0       = [p.R_w; 0; bb0];
    X0_fixed = X0;

    % [신규 추가] 수백만 번 호출되는 interp1 병목 제거 및 nearest 외삽 강제
    F_terrain = griddedInterpolant(x_t, y_t, 'linear', 'nearest');

    for i = 1:N
        xb  = x_arr(i);
        fun = @(X) ceq(X, xb, F_terrain, p); % x_t, y_t 대신 객체 전달
        % 1차: warm-start
        [sol, ~, ef] = fsolve(fun, X0, opts);

        if ef <= 0
            % 2차: 고정 초기값으로 재시도
            [sol2, ~, ef2] = fsolve(fun, X0_fixed, opts);
            if ef2 > 0; sol = sol2; ef = ef2; end
        end

        if ef > 0
            X0      = sol;
            R.y0(i) = sol(1); R.ar(i) = sol(2); R.bb(i) = sol(3);
            [Wf,Wm,Wr,Pb,CG] = wpos(sol, xb, p);   % v2: 5출력
            R.xwf(i)=Wf(1); R.ywf(i)=Wf(2);
            R.xwm(i)=Wm(1); R.ywm(i)=Wm(2);
            R.xwr(i)=Wr(1); R.ywr(i)=Wr(2);
            R.xpb(i)=Pb(1); R.ypb(i)=Pb(2);
            R.xcg(i)=CG(1); R.ycg(i)=CG(2);
        else
            R.ok(i) = false;
        end
    end

    n_fail      = sum(~R.ok);
    R.fail_rate = n_fail / N;

    if n_fail > 0 && n_fail < N
        % [BUG-6] 보간 후 fsolve 재시도:
        %   기존: 선형 보간값을 그대로 wpos에 전달 -> 역기구학 제약 미만족
        %         calc_dynamics에서 잘못된 법선력 산출
        %   수정: 보간값을 초기값으로 fsolve 재시도
        %         성공 시 R.ok=true 복원 / 실패 시 보간값 유지(R.ok=false 유지)
        R.y0 = fill_nan(R.y0); R.ar = fill_nan(R.ar); R.bb = fill_nan(R.bb);
        fi = find(~R.ok);
        n_recovered = 0;
        % [R-2] 복원용 fsolve 알고리즘도 'trust-region-dogleg'로 통일
        opts_retry = optimoptions('fsolve', ...
            'Display','off','TolFun',1e-8,'TolX',1e-8, ...
            'MaxIterations',200,'Algorithm','trust-region-dogleg');
        for i = fi
            xb_i  = x_arr(i);
            X0_i  = [R.y0(i); R.ar(i); R.bb(i)];
            % [수정] ceq 입력 인자 변경 사항(F_terrain)을 재시도 루프에도 똑같이 반영
            fun_i = @(X) ceq(X, xb_i, F_terrain, p); 
            [sol_r, ~, ef_r] = fsolve(fun_i, X0_i, opts_retry);
            if ef_r > 0
                R.y0(i)=sol_r(1); R.ar(i)=sol_r(2); R.bb(i)=sol_r(3);
                R.ok(i)    = true;
                n_recovered = n_recovered + 1;
            end
            sol_use = [R.y0(i); R.ar(i); R.bb(i)];
            [Wf,Wm,Wr,Pb,CG] = wpos(sol_use, xb_i, p);   % v2: 5출력
            R.xwf(i)=Wf(1); R.ywf(i)=Wf(2);
            R.xwm(i)=Wm(1); R.ywm(i)=Wm(2);
            R.xwr(i)=Wr(1); R.ywr(i)=Wr(2);
            R.xpb(i)=Pb(1); R.ypb(i)=Pb(2);
            R.xcg(i)=CG(1); R.ycg(i)=CG(2);
        end
        R.fail_rate = sum(~R.ok) / N;   % fail_rate 재계산
        fprintf('  [보간+재시도] %d/%d 포인트 실패 → %d개 복원\n', n_fail, N, n_recovered);
    elseif n_fail == N
        % [FIX-3] a_r/b_r 없는 모드에서도 안전한 오류 메시지
        error('kin_sim: 모든 포인트 수렴 실패. bogie_mode="%s", R_w=%.3f', ...
              bmode, p.R_w);
    end

    R.fail_idx = find(~R.ok);
end

function v = fill_nan(v)
    idx = 1:length(v); vld = ~isnan(v);
    if sum(vld) < 2; v(~vld) = 0; return; end
    % [R-3] 선형 외삽('linear') 대신 최근접 값 복사('nearest') 적용
    v = interp1(idx(vld), v(vld), idx, 'nearest', 'extrap');
end