function [Wf, Wm, Wr, Pb, CG] = wpos(X, xb, p)
% wpos  순기구학 — 바퀴 및 피벗 위치 계산  [v2]
%
% ══ v2 수정 내역 ════════════════════════════════════════════════
%
%  [FIX-CRITICAL] triangle / frame 모드 분기 추가
%      - 기존: p.a_r, p.b_r, p.c_b, p.d_b (linear 고정값)만 사용
%              → triangle/frame decode_x 결과가 wpos에 전혀 반영 안 됨
%              → JointOptSearch가 구조를 바꿔도 실제 기구학은 항상 linear
%      - 수정: rocker_mode / bogie_mode 분기로 실제 암 길이 계산
%
%  [추가] CG 출력 (5번째 반환값)
%      - kin_sim이 R.xcg/ycg를 채울 수 있도록 CG 위치 반환
%
% ══ 각도 규약 ════════════════════════════════════════════════════
%   Rocker triangle:
%     ang_pb = ar_eff + alpha_r/2  (Pb 방향, 전방+상향)
%     ang_wr = ar_eff - alpha_r/2  (Wr 방향, 후방)
%     Pb = P0 + L_r1 * [cos(ang_pb); sin(ang_pb)]
%     Wr = P0 - L_r2 * [cos(ang_wr); sin(ang_wr)]
%
%   Bogie triangle (수직 아래 기준):
%     ang_vert = -pi/2 + bb
%     Wf = Pb + L_b1 * [cos(ang_vert - beta_b/2); sin(ang_vert - beta_b/2)]
%     Wm = Pb + L_b2 * [cos(ang_vert + beta_b/2); sin(ang_vert + beta_b/2)]
%
% ══ 입력 ════════════════════════════════════════════════════════
%   X   : [y0; ar; bb]
%   xb  : Rocker pivot x 좌표 [m]
%   p   : 파라미터 구조체
%
% ══ 출력 ════════════════════════════════════════════════════════
%   Wf, Wm, Wr : 바퀴 위치 [2×1]
%   Pb         : Bogie pivot 위치 [2×1]
%   CG         : 차체 무게중심 위치 [2×1]

    y0 = X(1);  ar = X(2);  bb = X(3);

    %% ── 기본값 ──────────────────────────────────────────────────
    phi_r0   = 0;   if isfield(p,'phi_r0'),   phi_r0   = p.phi_r0;        end
    delta_pb = 0;   if isfield(p,'delta_pb'), delta_pb = p.delta_pb;      end
    if ~isfield(p,'rocker_mode'), p.rocker_mode = 'linear'; end
    if ~isfield(p,'bogie_mode'),  p.bogie_mode  = 'linear'; end
    if ~isfield(p,'CG_offset'),   p.CG_offset   = 0;        end
    if ~isfield(p,'h_CG'),        p.h_CG        = p.h_body * 0.5; end

    P0     = [xb; y0];
    ar_eff = ar + phi_r0;

    %% ── Rocker 기구학 ────────────────────────────────────────────
    switch lower(p.rocker_mode)

        case 'triangle'
            ang_pb = ar_eff - p.alpha_r/2;         
            ang_wr = ar_eff - pi + p.alpha_r/2;     
            Pb = P0 + p.L_r1 * [cos(ang_pb); sin(ang_pb)];
            Wr = P0 + p.L_r2 * [cos(ang_wr); sin(ang_wr)]; 

        case 'frame'
            % Rocker frame 해석:
            %   T_r: 수평 빔 길이 [m]
            %   j_r: 피벗(P0) 위치 비율 → 전방(Pb쪽) = (1-j_r)*T_r, 후방(Wr쪽) = j_r*T_r
            %   S_r1: Pb로 향하는 수직 지지대 길이 [m]
            %   th_r1: 지지대 각도 (Rocker arm 기준, 0=arm 방향, 90=수직하향)
            %   수직 지지대는 지면 방향(-n_r)으로 내려가서 바퀴 접촉점에 연결
            u_r = [cos(ar_eff); sin(ar_eff)];
            n_r = [-sin(ar_eff); cos(ar_eff)];   % 수직 방향 (위쪽 +)
            % 지지대: arm 방향(u_r) + 지면 방향(-n_r) 합성
            Pb  = P0 + (1-p.j_r)*p.T_r * u_r ...
                     + p.S_r1 * ( sin(p.th_r1)*u_r - cos(p.th_r1)*n_r);
            Wr  = P0 - p.j_r   *p.T_r * u_r ...
                     + p.S_r2 * (-sin(p.th_r2)*u_r - cos(p.th_r2)*n_r);

        otherwise   % 'linear' (v1 호환)
            u_r  = [cos(ar_eff); sin(ar_eff)];
            n_r  = [-sin(ar_eff); cos(ar_eff)];
            Pb   = P0 + p.a_r * u_r + delta_pb * n_r;
            Wr   = P0 - p.b_r * u_r;
    end

    %% ── Bogie 기구학 ─────────────────────────────────────────────
    switch lower(p.bogie_mode)

        case 'triangle'
            ang_vert = -pi/2 + bb;
            ang_wf   = ang_vert + p.beta_b/2;       
            ang_wm   = ang_vert - p.beta_b/2;      
            Wf = Pb + p.L_b1 * [cos(ang_wf); sin(ang_wf)];
            Wm = Pb + p.L_b2 * [cos(ang_wm); sin(ang_wm)];

        case 'frame'
            % Bogie frame 해석:
            %   T_b: 수평 빔 길이 [m], j_b: Pb 위치 비율
            %   S_b1: Wf 지지대, S_b2: Wm 지지대 (모두 아래로)
            %   bb: Bogie pivot 회전각 (수평 기준)
            u_b = [cos(bb); sin(bb)];
            n_b = [-sin(bb); cos(bb)];   % 수직 방향 (위쪽 +)
            Wf  = Pb + (1-p.j_b)*p.T_b * u_b ...
                     + p.S_b1 * ( sin(p.th_b1)*u_b - cos(p.th_b1)*n_b);
            Wm  = Pb - p.j_b   *p.T_b * u_b ...
                     + p.S_b2 * (-sin(p.th_b2)*u_b - cos(p.th_b2)*n_b);

        otherwise   % 'linear' (v1 호환)
            Wf   = Pb + p.c_b * [cos(bb); sin(bb)];
            Wm   = Pb - p.d_b * [cos(bb); sin(bb)];
    end

    %% ── CG 위치 ──────────────────────────────────────────────────
    u_h = [cos(ar_eff);  sin(ar_eff)];
    n_h = [-sin(ar_eff); cos(ar_eff)];
    CG  = P0 + p.CG_offset * u_h + p.h_CG * n_h;
end
