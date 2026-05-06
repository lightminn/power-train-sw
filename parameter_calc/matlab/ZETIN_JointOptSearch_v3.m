% ============================================================
% ZETIN_JointOptSearch_v3.m
% Rocker × Bogie 전체 파라미터 최적 탐색 — surrogateopt 버전 v3
%
% ══ 병렬 처리 및 간섭 제약 업데이트 통합 ════════════════════════
%  - [SECTION 4] calc_stability의 S.is_collision 결과를 받아 
%    차체가 지형에 걸릴 경우 강한 페널티(9 + 여유공간*20) 부여
%  - [SECTION 7] surrogateopt 및 ga 연산 시 UseParallel = true 로 변경
%    (수십만 번의 기구학 연산 병목 대폭 해소)
%
% ══ v3 수정 내역 (v2 대비) ══════════════════════════════════
%
%  ★ 확정 설계 스펙 반영 ★
%    R_w=200mm  mass=30kg  L_robot=1.2m
%    v_max=2.0m/s  v_stair=0.8m/s
%    Motor: D6374 150KV  Kt=0.055Nm/A  gear 5:1
%           I_peak=90A  -> tau_m_peak=4.95Nm
%           I_cont=50A  -> tau_m_cont=2.75Nm
%           wheel 피크 = 4.95x5x0.85 = 21.0Nm
%
%  [gear5:1-A] gear_ratio 30 → 5  ★파라미터 수정★
%      모든 모터 토크 산정이 gear 5:1 기준으로 재계산됨
%
%  [gear5:1-B] TAU_REF 0.60 → 4.26 Nm  ★목적함수 수정★
%      gear 5:1 기준 재산정:
%        tau_drive = 0.410Nm (정상 등판, F_drv /2 적용)
%        tau_impact = 3.851Nm (Hertz, v=0.8m/s, k=1e5N/m)
%        TAU_REF = 4.26Nm (기존 0.60은 gear30 기준 → 목적함수 7배 왜곡)
%
%  [gear5:1-C] F_drv /2 복원 (calc_dynamics)  ★물리 모델 수정★
%      v3-BUG-2에서 /2를 제거했으나 이는 오류:
%      2D 모델은 W=30kg 전체중량 사용 → 한쪽 3바퀴 분담 = /2 필요
%      /2 제거 시 tau_motor 2배 과대추정
%
%  [gear5:1-D] 탐색 하한 R_w 기준 상향  ★탐색 공간 수정★
%      L_r1/L_r2: 0.15/0.20 → 0.25/0.25m (R_w 이상)
%      L_b1/L_b2: 0.10/0.10 → 0.20/0.20m (R_w 이상, 바퀴간섭 방지)
%      c_b/d_b 하한: 0.08 → R_w=0.20m
%      get_cb_fwd/b_eff/a_eff 하한: 0.10 → R_w=0.20m
%
%  [gear5:1-E] WBOT_MAX 0.600 → 0.700m (탐색 다양성 확보)
%
%  [BUG-1] 물리 파라미터 전면 재산정
%      R_w:100->200mm / mass:유지(30kg) / h_body:150->300mm
%      motor_tau_peak:4.0->4.95Nm(D6374실측)
%      m_wheel:1.2->3.5kg / m_rocker:0.8->1.5kg / m_bogie:0.5->0.8kg
%      v_robot:1.0->0.8m/s(계단진입) / h_CG: x0.5->x0.55
%      TAU_REF:0.25->0.60Nm / WBOT:[200~300]->[400~600]mm
%      TOI_WARN:0.15->0.20 / W.stab:0.20->0.30 / W.tau:0.30->0.25
%      SN_REF:40->35dB / lb/ub 링크길이 x2 스케일
%      S 스케일팩터 4e-3->5e-3 m/deg / alpha_r≤40° / beta_b≤55°
%      j_r/j_b: [0.1~0.9]->[0.25~0.75]
%
%  [BUG-2] calc_dynamics F_drv /2 — 복원됨 (gear5:1-C 참조)
%      v3에서 /2 제거를 시도했으나 물리적 오류로 판명 → 복원
%
%  [BUG-3] decode_x S_r2/S_b2 스케일 4e-3->5e-3 m/deg
%      200mm 바퀴 기준: 40deg x 5e-3 = 200mm (x2 스케일)
%
%  [BUG-4] 탐색 경계 물리적 한계 적용
%      alpha_r 90°->40° (Rocker 수직 방지)
%      beta_b  90°->55° (Bogie 과도 개각 방지)
%      j_r/j_b 0.1->0.25 하한 (극단 편심 방지)
%
%  [BUG-5] calc_dynamics 충격 접촉시간 Hertz 이론 적용
%      기존: 2*R_w/v_robot = 0.5s (실제의 수백배)
%      수정: pi*sqrt(m_eff/k_contact), k=1e5 N/m (우레탄 200mm)
%
%  [BUG-6] kin_sim 보간 후 fsolve 재시도
%      보간값을 초기값으로 fsolve 재시도 -> 역기구학 제약 만족 복원
%
%  [BUG-7] 편중도 계산에서 liftoff 포인트 제외
%      들림 포인트(N=0 클리핑)가 mean에 포함되어 편중도 왜곡 수정
%
%  [BUG-8] gen_terrain real_stairs 내리막 에지 수정
%      덮어쓰기 순서 오류로 에지 뭉개지는 문제 수정
%
%  [BUG-9] calc_stability ratio_fm 방향 주석 명시
%
%  [BUG-10] calc_wbot frame 모드 WBOT 우회 방지
%      S*cos(th)만으로 WBOT 계산 시 긴 암이 좁게 통과하는 문제 수정
%
% ══ v4 추가 수정 (v2 원본 유지)════════════════════════════
%
%  [J-3] S_r2/S_b2 스케일 팩터 3e-3 → 4e-3
%      - 이전 최적해 S_r2≈300mm가 탐색 범위(270mm) 밖에 있었음
%      - 수정: 5deg→20mm, 90deg→360mm (이전 최적 포함)
%
%  [J-2+D-1] decode_x/calc_wbot/get_cb_fwd/Section8 frame sin→cos
%      - bb≈-π/2 기준: 바퀴 수평폭 = S*cos(th)  (이전 S*sin(th)와 26% 오차)
%      - calc_dynamics D-1 수정과 일관성 통일
%
% ══ v3 수정 내역 ════════════════════════════════════════════
%
%  [FIX-A] 탐색 공간 경계 단위 통일 — x(5)/x(11) 모두 deg
%      - frame S_r2/S_b2 = x * 4e-3 m/deg (v4에서 3e-3→4e-3 확장)
%
%  [FIX-B] 관성력 이중 cos 제거 (calc_dynamics)
%      - 기존: F_in_y = mass * ay_cg * cos(theta) → 이중 보정
%      - 수정: F_in_y = mass * ay_cg (수직 가속도 그대로)
%
%  [FIX-C] p.liftoff_max: objective 루프 외부에서 한 번만 설정
%
%  [FIX-D] kin_sim triangle bb0 = beta_b/2 (올바른 평지 초기값)
%
%  [FIX-E] 구동력: F_grade + F_roll 명시적 분리
%
%  [FIX-F] catch 블록 t 미정의 → terrains{ti} 사용
%
%  [FIX-G] ratio_rb 부호 보존: 절댓값 1e-3 미만만 방어
%
% ══ v2 수정 내역 ════════════════════════════════════════════
%  [1] 목적함수에 안정성 페널티 통합 (calc_stability 연동)
%  [2] 목적함수에 링크 관성 토크 반영 (calc_dynamics v2 연동)
%  [3] fail_rate를 모든 지형에서 누적 집계
%  [4] S/N비를 calc_metrics v3 (지형 기반 이상 궤적) 기준으로 업데이트
%  [5] 가중치 민감도 분석 스크립트 내장
%  [6] 이중 클리핑 제거 (lb/ub에서만 경계 관리)
%  [7] 목적함수 정규화 기준값 명시적 상수로 분리
%
% ══ 목적함수 구조 ═══════════════════════════════════════════
%
%   f = w_tau  · (τ_peak / TAU_REF)
%     + w_imbal· (imbal / 100)
%     + w_fail · fail_rate_total
%     + w_stab · stability_penalty
%     + w_sn   · (1 / (1 + SN_dB_mean / SN_REF))
%
%   강성 제약 (위반 시 즉시 페널티 반환):
%     - 차체 간섭 발생 시 (is_collision) → penalty 9 + clearance*20
%     - liftoff_ratio > LIFTOFF_MAX  → penalty 7
%     - fail_rate_total > FAIL_MAX   → penalty 5 + fail*5
%     - xs >= xe (구조 크기 초과)    → penalty 12
%     - W_bot 범위 위반              → penalty 8
%
% ══ 필요 파일 ════════════════════════════════════════════════
%   wpos.m, ceq.m, kin_sim.m (v3), gen_terrain.m
%   calc_dynamics.m  (v3), calc_stability.m (v2), calc_metrics.m (v3)
% ============================================================

clear; clc; close all;

% [신규 추가] 최적화 재현성을 위해 전역 난수 시드를 최상단에서 1회만 고정
rng('default'); 
rng(2026);

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);
addpath(fullfile(script_dir, 'functions'));

%% ═════════════════════════════════════════════════════════════
%% [SECTION 0]  실행 옵션
%% ═════════════════════════════════════════════════════════════
RUN_SENSITIVITY = true;    % true: 최적화 후 가중치 민감도 분석 실행
N_SENS_PERTURB  = 5;       % 민감도 분석: 가중치당 섭동 횟수 (±10% 격자)

%% ═════════════════════════════════════════════════════════════
%% [SECTION 1]  공통 파라미터
%% ═════════════════════════════════════════════════════════════
% [BUG-1] 확정 설계 스펙 기반 전면 재산정
%
% 계단 진입 토크 산정 (gear 5:1, rise=80mm, depth=500mm, theta=9.09deg):
%   W = 30x9.81 = 294.3N
%   F_grade  = 294.3xsin(9.09) = 46.4N (전체)
%   F_roll   = 0.02x294.3xcos(9.09) = 5.8N
%   F_drv_side = (46.4+5.8)/2 = 26.15N  (2D 모델: 한쪽 3바퀴 분담 /2)

% After (R_w=0.100 기준으로 수정)
%   tau_wheel/바퀴 = 26.15x0.100/3 = 0.872Nm
%   tau_motor = 0.872/(5x0.85) = 0.205Nm
%   tau_motor_impact = 209.4x0.100/3/(5x0.85) = 1.642Nm
%   TAU_REF = 0.205 + 1.642 = 1.847Nm → 1.85Nm

%   tau_peak_total ~ 4.26Nm -> TAU_REF = 4.26Nm
%   피크 안전율 = 4.95/4.26 = 1.16배  (충격 완화 필수)
%   연속 안전율 = 2.75/0.41 = 6.7배  (정상 등판: 충분)
%
% 바퀴 간격: 직경400mm -> WBOT_MIN=400mm, 폭 여유 -> WBOT_MAX=700mm
p0.R_w            = 0.100;    % 바퀴 반지름 [m] (지름 20cm로 수정)
p0.h_body         = 0.300;    % 차체 높이 [m]
p0.mass           = 30;       % 총 중량 [kg]
p0.g              = 9.81;
p0.obs_h          = 0.150;    % 단차 목표 높이 [m] (15cm로 하향 수정)
p0.mu             = 0.70;
p0.gear_ratio     = 5;        % ★ 기어비 5:1 (이전 30:1에서 수정)
p0.eta_gear       = 0.85;
p0.motor_tau_peak = 4.95;     % D6374 피크 토크 [Nm]
p0.motor_tau_cont = 2.75;     % D6374 연속 토크 [Nm]
p0.Kt             = 0.055;    % 토크상수 [Nm/A]
p0.m_wheel        = 3.5;      % 바퀴 질량 [kg]
p0.m_rocker_link  = 1.5;      % Rocker 링크 질량 [kg]
p0.m_bogie_link   = 0.8;      % Bogie  링크 질량 [kg]
p0.I_rocker_add   = 0;
p0.I_bogie_add    = 0;
p0.e_restitution  = 0.3;
p0.v_robot        = 0.8;      % 계단 진입 설계 속도 [m/s]
p0.v_max_flat     = 2.0;      % 평지 최대 속도 [m/s]
p0.step_thresh    = 5.0;
p0.phi_r0         = 0;
p0.delta_pb       = 0;
p0.CG_offset      = 0;
p0.h_CG           = p0.h_body * 0.55;

%% ═════════════════════════════════════════════════════════════
%% [SECTION 2]  목적함수 설계 상수 (한 곳에서 관리)
%% ═════════════════════════════════════════════════════════════
% ── 정규화 기준값 ───────────────────────────────────────────
% TAU_REF  : 모터 토크 기준값 [Nm]  (gear 5:1 기준 재산정)
%   tau_m_drive  = 0.410Nm (정상 등판, /2 적용)
%   tau_m_impact = 3.851Nm (Hertz, v=0.8m/s, k=1e5N/m)
%   tau_peak_total = 4.26Nm -> TAU_REF = 4.26Nm
%   (이전 0.60Nm은 gear 30:1 기준 → gear 5:1에서 목적함수 왜곡)

% R_w=100mm 기준 재산정:
%   tau_motor_drive  = 0.205Nm  (F_drv=26.15N, R_w=100mm, gear5:1)
%   tau_motor_impact = 1.642Nm  (Hertz, v=0.8m/s, k=1e5N/m, R_w=100mm)
%   TAU_REF = 1.85Nm5
TAU_REF = 1.85;

% IMBAL_REF
IMBAL_REF = 10; % 스케일 정상화

% SN_REF
SN_REF    = 35;

% ── 강성 제약 임계값 ────────────────────────────────────────
WBOT_MIN     = 0.400;   % 바퀴 직경 400mm → 최소 간격 (물리 간섭 방지)
WBOT_MAX     = 0.700;   % 로봇 폭 700mm 이하 (이전 600mm → 링크 다양성 확보)
FAIL_MAX     = 0.10;
LIFTOFF_MAX  = 0.02;
TOI_WARN     = 0.20;
% [HEIGHT-A] P0(Rocker pivot) 평지 기준 최대 높이 제약
%   목적: 지지대가 비현실적으로 길어져 죽마(stilts) 구조 방지
%   근거: P0 <= 500mm
P0_HEIGHT_MAX = 0.500;

% ── 목적함수 가중치 ─────────────────────────────────────────
% gear 5:1에서 충격 토크가 피크 토크의 92% → 토크 비중 유지
W.tau   = 0.25;
W.imbal = 0.20;
W.stab  = 0.30;
W.sn    = 0.15;
W.fail  = 0.10;

% 지형별 토크 가중치 (τ 항 내부 배분, 합 = 1.0)
W_terrain.stairs = 0.55;   % 실제 계단 (경진대회 핵심)
W_terrain.wood   = 0.20;   % 목재 블록
W_terrain.rough  = 0.15;   % 불규칙 노면
W_terrain.step   = 0.10;   % 단차

% ── 평가 해상도 ─────────────────────────────────────────────
N_PTS = 160;   % 지형당 포인트 수 (속도-정밀도 균형)

fprintf('=== ZETIN Rocker×Bogie surrogateopt v3 (200mm/30kg/D6374) ===\n');
fprintf('날짜: %s\n', char(datetime('now','Format','yyyy-MM-dd HH:mm')));
fprintf('스펙: R_w=%.0fmm  mass=%.0fkg  v_stair=%.1fm/s  v_max=%.1fm/s\n', ...
        p0.R_w*1000, p0.mass, p0.v_robot, p0.v_max_flat);
fprintf('D6374: Kt=%.3fNm/A  tau_peak=%.2fNm  tau_cont=%.2fNm\n', ...
        p0.Kt, p0.motor_tau_peak, p0.motor_tau_cont);
fprintf('TAU_REF=%.2fNm  WBOT=[%.0f~%.0f]mm  TOI_warn=%.2f\n', ...
        TAU_REF, WBOT_MIN*1000, WBOT_MAX*1000, TOI_WARN);
fprintf('가중치: τ=%.2f  imbal=%.2f  stab=%.2f  SN=%.2f  fail=%.2f\n\n', ...
        W.tau, W.imbal, W.stab, W.sn, W.fail);

%% ═════════════════════════════════════════════════════════════
%% [SECTION 3]  탐색 공간 경계 (14차원)
%% ═════════════════════════════════════════════════════════════
% [v3 FIX-A] 단위 통일: 모든 연속 파라미터를 SI 단위(m, rad)로 통일
%   - 이전: x(5)/x(11)을 triangle=deg, frame=mm 혼용 → 대리 모델 학습 오염
%   - 수정: x(5)/x(11)을 모두 deg로 통일, decode_x에서 모드별로 해석
%     triangle mode: alpha_r/beta_b [deg]
%     frame mode:    S_r2/S_b2 → x(5)/x(11)을 deg 범위(0~90)로 → decode에서 *3mm/deg
%                    10deg→30mm, 90deg→270mm 로 선형 스케일 (0.03~0.27m 범위)
%   - 공통 lb/ub 보장: 두 모드 모두 물리적으로 의미 있는 범위
%
% 경계 설계 (모두 [m] 또는 [deg]):
%   x(1): rocker_mode [1=triangle, 2=frame]
%   x(2): bogie_mode  [1=triangle, 2=frame]
%   x(3): L_r1 or T_r   [0.06~0.30 m]
%   x(4): L_r2 or S_r1  [0.06~0.40 m]
%   x(5): alpha_r[deg] or S_r2_scaled[deg→m]  [5~90 deg]
%   x(6): (미사용) or th_r1 [0~45 deg]
%   x(7): (미사용) or th_r2 [0~45 deg]
%   x(8): (미사용) or j_r   [0.1~0.9]
%   x(9):  L_b1 or T_b  [0.04~0.20 m]
%   x(10): L_b2 or S_b1  [0.04~0.20 m]
%   x(11): beta_b[deg] or S_b2_scaled[deg→m] [5~90 deg]
%   x(12): (미사용) or th_b1 [0~55 deg]
%   x(13): (미사용) or th_b2 [0~55 deg]
%   x(14): (미사용) or j_b   [0.1~0.9]

% [BUG-3][BUG-4] 200mm 바퀴 기준 링크 길이 x2, 개각 물리 제한
%   x(3): L_r1/T_r   [m]  0.25~0.55  (하한 0.15→0.25: R_w 이상 확보)
%   x(4): L_r2/S_r1  [m]  0.25~0.65  (하한 0.20→0.25: R_w 이상)
%   x(5): alpha_r[5~40deg] / S_r2_scale[deg->m, x5e-3]
%          40deg x 5e-3 = 200mm  alpha_r 상한 40도: 수직 암 구조 방지
%   x(8): j_r [0.25~0.75]
%   x(9) : L_b1/T_b  [m]  0.20~0.40  (하한 0.10→0.20: R_w 이상, 바퀴간섭 방지)
%   x(10): L_b2/S_b1 [m]  0.20~0.40  (하한 0.10→0.20: 동일)
%   x(11): beta_b[5~55deg] / S_b2_scale[deg->m, x5e-3]
%          55deg x 5e-3 = 275mm  beta_b 상한 55도: 과도 개각 방지
%   x(14): j_b [0.25~0.75]

% 피벗 비율(x8, x14) 0~1 확장 / 각도 및 지지대(x5, x11) 60~160 확장
% [수정] 지나친 장대형 구조를 막기 위해 링크 최대 길이(ub)를 현실적으로 축소
lb = [2,    2, ...
      0.20, 0.15,  60,  0,  0,  0.30, ...
      0.15, 0.15,  60,  0,  0,  0.30];
ub = [2,    2, ...
      0.45, 0.35, 160, 35, 35,  0.70, ...
      0.35, 0.25, 160, 40, 40,  0.70];

int_vars = [1, 2];

%% ═════════════════════════════════════════════════════════════
%% [SECTION 4]  목적함수 (단일 평가)
%% ═════════════════════════════════════════════════════════════
function f = objective(x, p0, N_PTS, WBOT_MIN, WBOT_MAX, ...
                       FAIL_MAX, LIFTOFF_MAX, TOI_WARN, ...
                       TAU_REF, IMBAL_REF, SN_REF, W, W_terrain, P0_HEIGHT_MAX)

    %% ── 파라미터 구조체 변환 ─────────────────────────────────
    p = decode_x(x, p0);
    if isempty(p), f = 10; return; end

    %% ── W_bot 제약 ───────────────────────────────────────────
    Wbot = calc_wbot(p);
    % [수정] 알고리즘의 꼼수(Loophole)를 막기 위해 기각 페널티를 50점 이상으로 대폭 상향
    if Wbot < WBOT_MIN || Wbot > WBOT_MAX
        f = 50 + abs(Wbot - (WBOT_MIN+WBOT_MAX)/2); return;
    end

    %% ── [HEIGHT-B] P0 높이 제약 (평지 기준 해석적 계산) ──────────
    % 평지(ar=0)에서 Wr 접지 조건으로부터 y0를 구조 파라미터로 계산
    %   triangle: Wr_y = y0 + L_r2*sin(-pi+alpha_r/2) = R_w
    %             → y0_flat = R_w + L_r2*sin(alpha_r/2)
    %   frame:    Wr_y = y0 - S_r2*cos(th_r2) = R_w
    %             → y0_flat = R_w + S_r2*cos(th_r2)
    % 이 값이 P0_HEIGHT_MAX 초과 시 즉시 기각 (페널티 8 + 초과량 비례)
    y0_flat = calc_P0_height_flat(p);
    if y0_flat > P0_HEIGHT_MAX
        f = 50 + (y0_flat - P0_HEIGHT_MAX) * 10; return;
    end

    %% ── [신규 추가] 중간-뒷바퀴 간섭(Overlap) 제약 ───────────────
    % 뒷바퀴(Wr)와 중간바퀴(Wm)의 수평 거리가 
    % 바퀴 지름(0.20m) + 여유(0.05m) = 0.25m 미만이면 즉시 기각
    dist_mr = get_a_eff(p) + get_b_eff(p) - p.d_b;
    if dist_mr < 0.250
        f = 50 + abs(0.250 - dist_mr) * 10; return;
    end

    %% ── [신규 추가] 평지(Flat) 단위 테스트 (각도 규약 검증) ────────
    % 최적화 전 평지에서 기구학이 풀리는지 1차 검증하여 잘못된 각도 조합을 조기 기각
    [flat_x, flat_y_raw] = gen_terrain('flat', p);
    flat_y_env = calc_envelope(flat_x, flat_y_raw, p.R_w);
    test_R = kin_sim(0, flat_x, flat_y_env, p);
    if ~test_R.ok(1)
        f = 60; return; % 평지에서 넘어지면 최악의 페널티
    end

    %% ── 4종 지형 평가 ────────────────────────────────────────
    terrains    = {'real_stairs','wood_block','rough','step'};
    t_weights   = [W_terrain.stairs, W_terrain.wood, ...
                   W_terrain.rough,  W_terrain.step];
    n_t         = length(terrains);

    tau_vals    = zeros(1, n_t);
    sn_vals     = zeros(1, n_t);
    imbal_vals  = zeros(1, n_t);
    fail_pts    = zeros(1, n_t);
    total_pts   = zeros(1, n_t);
    toi_min_all = ones(1, n_t);
    liftoff_all = zeros(1, n_t);

    % calc_stability risk_level 판정 기준을 JointOptSearch와 일치시킴
    % (루프 안에서 반복 대입하지 않도록 한 번만 설정)
    p.liftoff_max = LIFTOFF_MAX;

    for ti = 1:n_t
        t = terrains{ti};
        try
            %% ─ 지형 생성 및 Minkowski 팽창 적용 ───────────
            [x_t, y_t_raw] = gen_terrain(t, p);
            y_t_env = calc_envelope(x_t, y_t_raw, p.R_w); % 팽창된 궤적 생성
            
            b_eff = get_b_eff(p);
            a_eff = get_a_eff(p);
            cb    = get_cb_fwd(p);
            xs    = x_t(1)   + b_eff + 0.05;
            xe    = x_t(end) - (a_eff + cb) - 0.05;
            if xs >= xe, f = 12; return; end
            xa = linspace(xs, xe, N_PTS);

            %% ─ 기구학 시뮬레이션 (팽창된 지형 y_t_env 탑승) ──
            R = kin_sim(xa, x_t, y_t_env, p);
            fail_pts(ti)  = sum(~R.ok);
            total_pts(ti) = length(xa);

            %% ─ 동역학 계산 (팽창된 지형 y_t_env 사용) ──────
            D = calc_dynamics(R, xa, x_t, y_t_env, p);
            tau_vals(ti) = D.stair_torque_peak;

            %% ─ 안정성 및 간섭 분석 (원본 지형 y_t_raw, 팽창 지형 둘 다 전달) ──
            S = calc_stability(R, xa, x_t, y_t_raw, y_t_env, p);
            toi_min_all(ti) = S.min_TOI;
            liftoff_all(ti) = S.liftoff_ratio;

            %% ─ 차체 간섭(Collision) 제약 (즉시 기각) [추가] ─────
            if isfield(S, 'is_collision') && S.is_collision
                f = 9 + abs(S.min_clearance) * 20; % 파고든 깊이에 비례한 페널티
                return;
            end

            %% ─ 들림 강성 제약 (즉시 기각) ─────────────────
            % 어느 지형에서든 들림이 LIFTOFF_MAX 초과하면 기각
            if S.liftoff_ratio > LIFTOFF_MAX
                f = 7 + S.liftoff_ratio * 3;
                return;
            end

            %% ─ 법선력 편중도 [BUG-7] ──────────────────────
            % 들림(liftoff) 포인트를 제외하고 유효 포인트만으로 편중도 계산
            % 기존: liftoff N=0이 mean을 끌어내려 편중도 왜곡
            % [추가 수정] 기구학 수렴에 실패한 가짜 데이터(~R.ok)도 완벽히 마스킹하여 제외
            valid_d = ~D.liftoff_r & ~D.liftoff_f & R.ok;
            if sum(valid_d) > 5
                nm_ = [mean(D.Nr(valid_d)), mean(D.Nm(valid_d)), mean(D.Nf(valid_d))];
            else
                nm_ = [mean(D.Nr), mean(D.Nm), mean(D.Nf)];
            end
            if mean(nm_) > 0.5
                imbal_vals(ti) = (max(nm_)-min(nm_)) / mean(nm_) * 100;
            end

            %% ─ S/N비 (팽창된 지형 사용) ───────────
            M = calc_metrics(xa, x_t, y_t_env, R, p);
            sn_vals(ti) = M.SN_dB;   % var(err_demeaned) 기반

        catch ME
            % [FIX-F] t가 할당된 후 예외 발생 시만 출력 (미할당 시 terrains{ti} 사용)
            t_name = terrains{ti};
            if ~contains(ME.message, '수렴 실패')
                fprintf('  [예외] %s: %s\n', t_name, ME.message);
            end
            f = 10;
            return;
        end
    end

    %% ── 전체 지형 통합 fail_rate 계산 [v2: 전 지형 집계] ────
    fail_rate_total = sum(fail_pts) / max(sum(total_pts), 1);
    if fail_rate_total > FAIL_MAX
        f = 5 + fail_rate_total * 5;
        return;
    end

    %% ── 각 항 정규화 및 합산 ────────────────────────────────

    % ① 토크 항: 지형 가중 평균 토크 / TAU_REF
    tau_weighted = sum(t_weights .* tau_vals);
    tau_norm     = tau_weighted / TAU_REF;

    % ② 편중도 항: 실계단 기준 (가장 중요한 지형)
    %    → 전 지형 가중 평균으로 확장
    imbal_weighted = sum(t_weights .* imbal_vals);
    imbal_norm     = imbal_weighted / IMBAL_REF;

    % ③ 안정성 페널티 항
    %    TOI_min + ZMP이탈률 복합 패널티
    global_toi_min = min(toi_min_all);
    global_liftoff = max(liftoff_all);

    % ③ 안정성 항 [연속형 보상-페널티 모델로 개편]
    % 알고리즘이 0.5(완벽한 중앙 안정)에 가까워지도록 끊임없이 최적화하게 만듭니다.
    
    % TOI 패널티: 0.5(완벽한 중앙 안정성)에 가까워지도록 연속적 페널티 부여
    if global_toi_min >= TOI_WARN
        toi_penalty = (0.5 - global_toi_min) * 2; % 0.5일 때 0점
    else
        toi_penalty = 0.6 + ((TOI_WARN - global_toi_min) / TOI_WARN) * 5; % 위험 구역 폭증
    end

    % liftoff 패널티도 연속형으로 부여
    liftoff_penalty = (global_liftoff / LIFTOFF_MAX) * 3;
    stab_penalty = max(toi_penalty, liftoff_penalty);

    % ④ S/N비 항 
    sn_mean = sum(t_weights .* sn_vals);
    sn_norm = 1 / (1 + max(sn_mean, 0) / SN_REF);

    % ⑤ fail_rate 항 [페널티 민감도 10배 상향]
    fail_norm = fail_rate_total * 10;
    %% ── 최종 목적함수 합산 ──────────────────────────────────
    f = W.tau   * tau_norm   ...
      + W.imbal * imbal_norm ...
      + W.stab  * stab_penalty ...
      + W.sn    * sn_norm    ...
      + W.fail  * fail_norm;

    f = max(f, 0);
end

%% ═════════════════════════════════════════════════════════════
%% [SECTION 5]  헬퍼 함수
%% ═════════════════════════════════════════════════════════════

function p = decode_x(x, p0)
% x 벡터 → 파라미터 구조체
% [v3] 단위 통일: x(5)/x(11)을 deg로 통일
%   triangle: alpha_r/beta_b [deg → rad]
%   frame:    S_r2/S_b2 = x(5)*3mm/deg  (5deg→15mm, 90deg→270mm)
%             → 0.015~0.270m 범위, 물리적으로 적절
    p  = p0;
    rm = round(x(1));
    bm = round(x(2));

     % Bogie
    switch bm
        case 1   % triangle
            p.bogie_mode = 'triangle';
            p.L_b1   = x(9);
            p.L_b2   = x(10);
            p.beta_b = deg2rad(x(11));   
            p.c_b    = p.L_b1 * abs(sin(p.beta_b/2));
            p.d_b    = p.L_b2 * abs(sin(p.beta_b/2));
            h_bogie_drop = p.L_b1 * cos(-pi/2 + p.beta_b/2); % 평지 하강 높이

        case 2   % frame
p.bogie_mode = 'frame';
            p.T_b   = x(9);
            p.S_b1  = x(10);
            p.S_b2  = x(11) * 5e-3; % [옵션 B] 알고리즘이 맘대로 고른 비대칭 길이 그대로 사용
            p.th_b1 = deg2rad(x(12));
            p.th_b2 = deg2rad(x(13));
            p.j_b   = x(14);
            
            % [수학적 마법] Bogie가 비대칭일 때 평지에서 스스로 기울어지는 틸팅 각도(bb_0) 산출
            N_bb = p.S_b1*cos(p.th_b1) - p.S_b2*cos(p.th_b2);
            D_bb = p.T_b + p.S_b1*sin(p.th_b1) + p.S_b2*sin(p.th_b2);
            bb_0 = atan2(N_bb, D_bb);
            
            % 틸팅된 상태(bb_0)에서 Pb 조인트의 실제 수직 강하량 계산
            h_bogie_drop = -(1-p.j_b)*p.T_b*sin(bb_0) + p.S_b1*cos(p.th_b1 + bb_0);
            
            % bb_0 상태를 기준으로 한 실질적인 수평 투영폭 계산
            Wf_c = abs( (1-p.j_b)*p.T_b*cos(bb_0) + p.S_b1*sin(p.th_b1 + bb_0) );
            Wm_c = abs( p.j_b*p.T_b*cos(bb_0) + p.S_b2*sin(p.th_b2 - bb_0) );
            p.c_b = max(Wf_c, p0.R_w);
            p.d_b = max(Wm_c, p0.R_w);

        otherwise
            p = []; return;
    end

        % Rocker
    switch rm
        case 1   % triangle
            p.rocker_mode = 'triangle';
            p.L_r1 = x(3); 
            p.L_r2 = x(4); 
            p.alpha_r = deg2rad(x(5));
        case 2   % frame
            p.rocker_mode = 'frame';
            p.T_r   = x(3);
            p.S_r1  = x(4);
            p.th_r1 = deg2rad(x(6));
            p.th_r2 = deg2rad(x(7));
            p.j_r   = x(8);
            
            % [핵심] 차체와 Rocker 메인 빔(T_r)이 완벽히 수평(ar=0)이 되기 위한 조건
            % 뒷바퀴로 내려가는 총 높이 = (앞 지지대 높이) + (위에서 구한 Bogie 평지 높이)
            h_rocker_front = p.S_r1 * cos(p.th_r1);
            h_total_drop   = h_rocker_front + h_bogie_drop;
            
            % 수평을 유지하기 위해 뒷다리(S_r2) 길이를 강제로 덮어씌움!
            p.S_r2 = h_total_drop / cos(p.th_r2);
            
        otherwise
            p = []; return;
    end
end



   

function Wbot = calc_wbot(p)
% [BUG-10] frame 모드: 수평 투영 폭과 암 길이 합 중 큰 값 사용
%   기존: S*cos(th)만 계산 -> th가 크면 긴 암이 좁게 통과하는 우회 가능
%   수정: max(투영폭, 암길이합*0.7) -> 보수적 WBOT 적용
    switch lower(p.bogie_mode)
        case 'triangle'
            Wbot = (p.L_b1 + p.L_b2) * sin(p.beta_b/2);
        case 'frame'
            Wbot_proj = p.S_b1*abs(cos(p.th_b1)) + p.S_b2*abs(cos(p.th_b2));
            Wbot_raw  = p.S_b1 + p.S_b2;
            Wbot = max(Wbot_proj, Wbot_raw * 0.7);
        otherwise
            Wbot = p.c_b + p.d_b;
    end
end

function cb = get_cb_fwd(p)
    switch lower(p.bogie_mode)
        case 'triangle', cb = p.L_b1 * abs(sin(p.beta_b/2));
        case 'frame',    cb = p.S_b1*abs(cos(p.th_b1));
        otherwise,       cb = p.c_b;
    end
    cb = max(cb, p.R_w);   % 하한 = R_w (바퀴 반지름 이상: 이전 0.10m)
end

function b = get_b_eff(p)
    switch lower(p.rocker_mode)
        case 'linear',   b = p.b_r;
        case 'triangle', b = p.L_r2 * abs(cos(p.alpha_r/2));
        case 'frame',    b = p.j_r*p.T_r + p.S_r2*abs(sin(p.th_r2));
        otherwise,       b = 0.40;
    end
    b = max(b, p.R_w);   % 하한 = R_w (이전 0.10m)
end

function a = get_a_eff(p)
    switch lower(p.rocker_mode)
        case 'linear',   a = p.a_r;
        case 'triangle', a = p.L_r1 * abs(cos(p.alpha_r/2));
        case 'frame',    a = (1-p.j_r)*p.T_r + p.S_r1*abs(sin(p.th_r1));
        otherwise,       a = 0.35;
    end
    a = max(a, p.R_w);   % 하한 = R_w (이전 0.10m)
end

function y0 = calc_P0_height_flat(p)
% calc_P0_height_flat  평지(ar=0) 기준 P0(Rocker pivot) 높이를 해석적으로 계산
%
% [HEIGHT-B 상세]
%   P0 높이(y0)는 역기구학이 풀어주지만, 탐색 단계에서 역기구학 없이
%   구조 파라미터만으로 y0를 추정하는 함수.
%
%   원리: 평지 ar=0에서 Wr 접지 조건 Wr_y = R_w 를 이용해 y0를 역산
%
%   ▶ Rocker triangle (ar=0):
%     ang_wr = -pi + alpha_r/2
%     Wr_y   = y0 + L_r2 * sin(ang_wr) = R_w
%     y0     = R_w - L_r2 * sin(-pi + alpha_r/2)
%            = R_w + L_r2 * sin(alpha_r/2)   [sin(pi-x)=sin(x)]
%
%   ▶ Rocker frame (ar=0, u_r=[1;0], n_r=[0;1]):
%     Wr = P0 - j_r*T_r*u_r + S_r2*(-sin(th_r2)*u_r - cos(th_r2)*n_r)
%     Wr_y = y0 - S_r2*cos(th_r2) = R_w
%     y0   = R_w + S_r2 * cos(th_r2)
%
%   ▶ 기타(linear): a_r, b_r 기반 고정 추정값 사용
%
    switch lower(p.rocker_mode)
        case 'triangle'
            % sin(pi - alpha_r/2) = sin(alpha_r/2)
            y0 = p.R_w + p.L_r2 * sin(p.alpha_r / 2);
        case 'frame'
            % 평지 ar=0 → n_r 방향이 수직 위 → Wr의 y = y0 - S_r2*cos(th_r2)
            y0 = p.R_w + p.S_r2 * cos(p.th_r2);
        otherwise   % linear
            % linear 모드는 b_r이 수평 → P0는 R_w에 근접
            y0 = p.R_w;
    end
    y0 = max(y0, p.R_w);   % 최솟값 보정
end

%% ═════════════════════════════════════════════════════════════
%% [SECTION 6]  목적함수 래퍼 및 이력 기록
%% ═════════════════════════════════════════════════════════════
obj_fn = @(x) objective(x, p0, N_PTS, WBOT_MIN, WBOT_MAX, ...
                        FAIL_MAX, LIFTOFF_MAX, TOI_WARN, ...
                        TAU_REF, IMBAL_REF, SN_REF, W, W_terrain, P0_HEIGHT_MAX);

%% ═════════════════════════════════════════════════════════════
%% [SECTION 7]  최적화 실행
%% ═════════════════════════════════════════════════════════════
has_surrogate = license('test','optimization_toolbox') && ...
                exist('surrogateopt','file') == 2;
tic_total = tic;

if has_surrogate
    fprintf('surrogateopt 실행 중 (MaxFunctionEvaluations=4000, 병렬 처리)...\n\n');
    % [v4] UseParallel = true 로 변경하여 다중 코어 병렬 처리 활성화
    opts = optimoptions('surrogateopt', ...
        'MaxFunctionEvaluations', 4000, ...
        'Display',     'iter', ...
        'UseParallel', true);

    [x_opt, f_opt, exit_flag, output] = surrogateopt( ...
        obj_fn, lb, ub, int_vars, opts);

    fprintf('\n[surrogateopt 완료] 코드:%d  평가:%d\n', ...
            exit_flag, output.funccount);
else
    fprintf('[경고] surrogateopt 미발견 → ga()로 대체\n\n');
    % [v4] UseParallel = true 로 변경하여 병렬 처리 활성화
    ga_opts = optimoptions('ga', ...
        'MaxGenerations',    80, ...
        'PopulationSize',    60, ...
        'FunctionTolerance', 1e-4, ...
        'UseParallel',       true, ... 
        'Display',           'iter');
    [x_opt, f_opt, exit_flag, output] = ga( ...
        obj_fn, 14, [],[],[],[], lb, ub, [], int_vars, ga_opts);
    fprintf('\n[ga 완료] 세대:%d  평가:%d\n', ...
            output.generations, output.funccount);
end

elapsed = toc(tic_total);
fprintf('총 소요 시간: %.1f분\n\n', elapsed/60);

%% ═════════════════════════════════════════════════════════════
%% [SECTION 8]  최적 파라미터 복원 및 출력
%% ═════════════════════════════════════════════════════════════
p_opt = decode_x(x_opt, p0);

fprintf('%s\n', repmat('═',1,60));
fprintf('  최적 구조\n');
fprintf('%s\n', repmat('─',1,60));
fprintf('Rocker mode : %s\n', p_opt.rocker_mode);
switch lower(p_opt.rocker_mode)
    case 'triangle'
        fprintf('  L_r1=%.1fmm  L_r2=%.1fmm  α_r=%.1f°\n', ...
                p_opt.L_r1*1000, p_opt.L_r2*1000, rad2deg(p_opt.alpha_r));
    case 'frame'
        fprintf('  T_r=%.1fmm  S_r1=%.1fmm  S_r2=%.1fmm\n', ...
                p_opt.T_r*1000, p_opt.S_r1*1000, p_opt.S_r2*1000);
        fprintf('  θ_r1=%.1f°  θ_r2=%.1f°  j_r=%.2f\n', ...
                rad2deg(p_opt.th_r1), rad2deg(p_opt.th_r2), p_opt.j_r);
end
fprintf('Bogie mode  : %s\n', p_opt.bogie_mode);
switch lower(p_opt.bogie_mode)
    case 'triangle'
        Wb = (p_opt.L_b1+p_opt.L_b2)*sin(p_opt.beta_b/2);
        fprintf('  L_b1=%.1fmm  L_b2=%.1fmm  β_b=%.1f°  W_bot=%.1fmm\n', ...
                p_opt.L_b1*1000, p_opt.L_b2*1000, rad2deg(p_opt.beta_b), Wb*1000);
    case 'frame'
        Wf_=p_opt.S_b1*abs(cos(p_opt.th_b1));   % [D-1] bb≈-π/2 기준
        Wm_=p_opt.S_b2*abs(cos(p_opt.th_b2));
        fprintf('  T_b=%.1fmm  S_b1=%.1fmm  S_b2=%.1fmm\n', ...
                p_opt.T_b*1000, p_opt.S_b1*1000, p_opt.S_b2*1000);
        fprintf('  θ_b1=%.1f°  θ_b2=%.1f°  j_b=%.2f  W_bot=%.1fmm\n', ...
                rad2deg(p_opt.th_b1), rad2deg(p_opt.th_b2), p_opt.j_b, (Wf_+Wm_)*1000);
end
y0_opt = calc_P0_height_flat(p_opt);
fprintf('P0 평지높이 : %.1fmm  (제약: ≤%.0fmm)\n', y0_opt*1000, P0_HEIGHT_MAX*1000);
fprintf('목적함수 값 : %.4f\n', f_opt);
fprintf('%s\n\n', repmat('═',1,60));

%% ═════════════════════════════════════════════════════════════
%% [SECTION 9]  최종 검증 시뮬레이션 (4종 지형 + 전 모듈)
%% ═════════════════════════════════════════════════════════════
terrains_v = {'real_stairs','wood_block','rough','step'};
terrain_nm = {'실제 계단','목재 블록','불규칙','단차'};
results_v  = struct();

fprintf('최종 검증 시뮬레이션 (calc_stability + calc_metrics v3)...\n');
fail_pts_total = 0;
pts_total      = 0;
p_opt.liftoff_max = LIFTOFF_MAX;   % 위험 판정 기준 일관성 (한 번만 설정)

for ti = 1:4
    t       = terrains_v{ti};
    [x_t, y_t_raw] = gen_terrain(t, p_opt);
    y_t_env = calc_envelope(x_t, y_t_raw, p_opt.R_w); % 팽창 궤적 적용
    
    b_eff = get_b_eff(p_opt);
    a_eff = get_a_eff(p_opt);
    cb    = get_cb_fwd(p_opt);
    xs = x_t(1)   + b_eff + 0.05;
    xe = x_t(end) - (a_eff + cb) - 0.05;
    xa = linspace(xs, xe, 200);
    
    R = kin_sim(xa, x_t, y_t_env, p_opt);
    D = calc_dynamics(R, xa, x_t, y_t_env, p_opt);
    S = calc_stability(R, xa, x_t, y_t_raw, y_t_env, p_opt); % 6개 인자 정상 전달!
    M = calc_metrics(xa, x_t, y_t_env, R, p_opt);
    fail_pts_total = fail_pts_total + sum(~R.ok);
    pts_total      = pts_total + length(xa);

    % [BUG-7] 검증부도 liftoff 제외
    valid_v = ~D.liftoff_r & ~D.liftoff_f;
    if sum(valid_v) > 5
        nm_ = [mean(D.Nr(valid_v)),mean(D.Nm(valid_v)),mean(D.Nf(valid_v))];
    else
        nm_ = [mean(D.Nr),mean(D.Nm),mean(D.Nf)];
    end
    im_  = 0;
    if mean(nm_) > 0.5
        im_ = (max(nm_)-min(nm_)) / mean(nm_) * 100;
    end

    results_v(ti).terrain    = t;
    results_v(ti).tau_peak   = D.stair_torque_peak;
    results_v(ti).tau_link   = D.tau_link_ratio;
    results_v(ti).imbal      = im_;
    results_v(ti).fail_rate  = R.fail_rate;
    results_v(ti).toi_min    = S.min_TOI;
    results_v(ti).risk_level = S.risk_level;
    results_v(ti).sn_dB      = M.SN_dB;
    if isfield(S, 'min_clearance'), results_v(ti).clearance = S.min_clearance; end
    results_v(ti).R = R; results_v(ti).D = D;
    results_v(ti).S = S; results_v(ti).M = M;
    results_v(ti).x_arr = xa;
    results_v(ti).x_t   = x_t;
    % [수정] 변수명을 y_t_raw로 변경하고, 팽창 지형도 함께 저장
    results_v(ti).y_t   = y_t_raw; 
    results_v(ti).y_t_env = y_t_env;

    % gear 5:1 기준: 피크/연속 안전율 모두 표시
    sf_cont = p_opt.motor_tau_cont / max(D.stair_torque_peak, 1e-9);
    sf_peak = p_opt.motor_tau_peak / max(D.stair_torque_peak, 1e-9);
    peak_warn = '';
    if sf_peak < 1.2, peak_warn = ' ⚠️PEAK!'; end
    
    col_warn = '';
    if isfield(S, 'is_collision') && S.is_collision
        col_warn = ' (간섭!)';
    end

    fprintf('  [%s]  tau=%.3fNm(SF_cont=x%.2f SF_peak=x%.2f%s 링크%.0f%%)  imbal=%.1f%%  TOI=%.3f[%s]  SN=%.1fdB  fail=%.1f%%%s\n', ...
            terrain_nm{ti}, D.stair_torque_peak, sf_cont, sf_peak, peak_warn, ...
            D.tau_link_ratio*100, im_, S.min_TOI, S.risk_level, M.SN_dB, R.fail_rate*100, col_warn);
end

fprintf('\n  전 지형 통합 fail_rate: %.2f%%\n\n', ...
        fail_pts_total/pts_total*100);

%% ═════════════════════════════════════════════════════════════
%% [SECTION 10]  파라미터 저장
%% ═════════════════════════════════════════════════════════════
save_path = fullfile(script_dir, 'zetin_optimal_params_v3.mat');
save(save_path, 'p_opt', 'x_opt', 'f_opt', 'results_v', 'elapsed', ...
     'lb', 'ub', 'int_vars', 'W', 'W_terrain', ...
     'TAU_REF', 'IMBAL_REF', 'SN_REF', 'TOI_WARN', 'FAIL_MAX', 'LIFTOFF_MAX');
fprintf('최적 파라미터 저장: %s\n\n', save_path);

%% ═════════════════════════════════════════════════════════════
%% [SECTION 11]  시각화
%% ═════════════════════════════════════════════════════════════
t_cols = {[0.20 0.55 0.35],[0.80 0.50 0.15], ...
          [0.25 0.40 0.80],[0.70 0.25 0.20]};
W_tot = p_opt.mass * p_opt.g;

%% ── Figure 1: 법선력 + TOI (2행 × 4열) ──────────────────────
figure('Name','최적 구조 검증 — 법선력 & 안정성', ...
       'Position',[20 20 1500 840], 'Color','w');

for ti = 1:4
    rv  = results_v(ti);
    D   = rv.D; S = rv.S; x = rv.x_arr;
    col = t_cols{ti};

    % 상단: 법선력 + 들림 구간
    subplot(2,4,ti); hold on;
    lft = D.liftoff_r | D.liftoff_f;
    if any(lft)
        % [BUG-FIX] 불연속 논리 인덱스를 fill에 바로 넘기면 polygon이
        %   비연속 점들을 지그재그로 연결 → 연속 구간 단위로 분리해서 그림
        yl_tmp = [0, W_tot*0.8];
        lft_diff = diff([false, lft, false]);
        seg_starts = find(lft_diff == 1);
        seg_ends   = find(lft_diff == -1) - 1;
        for sg = 1:length(seg_starts)
            xs_seg = x(seg_starts(sg):seg_ends(sg));
            fill([xs_seg, fliplr(xs_seg)], ...
                 [repmat(yl_tmp(1),1,numel(xs_seg)), ...
                  repmat(yl_tmp(2),1,numel(xs_seg))], ...
                 [1 0.85 0.85], 'EdgeColor','none', 'FaceAlpha',0.4, ...
                 'HandleVisibility','off');
        end
    end
    plot(x, D.Nr, '-', 'Color',[0.10 0.45 0.75], 'LineWidth',1.5, 'DisplayName','Nr');
    plot(x, D.Nm, '-', 'Color',[0.10 0.60 0.25], 'LineWidth',1.5, 'DisplayName','Nm');
    plot(x, D.Nf, '-', 'Color',[0.85 0.40 0.10], 'LineWidth',1.5, 'DisplayName','Nf');
    yline(W_tot/3, '--k', '균등', 'LineWidth',0.8, 'FontSize',7, ...
          'LabelVerticalAlignment','bottom', 'HandleVisibility','off');
    title({terrain_nm{ti}, ...
           sprintf('imbal=%.1f%%  liftoff=%d  [%s]', ...
           rv.imbal, sum(lft), rv.risk_level)}, ...
           'FontSize',8, 'FontWeight','bold');
    if ti==1, legend('FontSize',6,'Location','northeast'); end
    xlabel('X [m]','FontSize',8); ylabel('N [N]','FontSize',8);
    grid on; box on;

    % 하단: TOI + 경고 구간
    subplot(2,4,ti+4); hold on;
    warn_zone = S.TOI < TOI_WARN;
    if any(warn_zone)
        % [BUG-FIX] 연속 구간 단위로 분리 fill
        wz_diff = diff([false, warn_zone, false]);
        wz_starts = find(wz_diff == 1);
        wz_ends   = find(wz_diff == -1) - 1;
        for sg = 1:length(wz_starts)
            xs_seg = x(wz_starts(sg):wz_ends(sg));
            fill([xs_seg, fliplr(xs_seg)], ...
                 [repmat(-0.3,1,numel(xs_seg)), repmat(1.1,1,numel(xs_seg))], ...
                 [1 0.92 0.82], 'EdgeColor','none', 'FaceAlpha',0.5, ...
                 'HandleVisibility','off');
        end
    end
    plot(x, S.TOI,       '-',  'Color',col,     'LineWidth',2.0, 'DisplayName','TOI');
    plot(x, S.TOI_front, '--', 'Color',col*0.7, 'LineWidth',0.9, 'DisplayName','TOI\_fwd');
    plot(x, S.TOI_rear,  ':',  'Color',col*0.7, 'LineWidth',0.9, 'DisplayName','TOI\_rr');
    yline(0,       'r-',  '전복', 'LineWidth',1.5, 'FontSize',7);
    yline(TOI_WARN,'r--', '경고', 'LineWidth',1.0, 'FontSize',7);
    ylim([-0.3, 1.15]);
    title(sprintf('%s  TOI_min=%.3f', terrain_nm{ti}, S.min_TOI), ...
          'FontSize',8, 'FontWeight','bold');
    if ti==1, legend('FontSize',6,'Location','southeast'); end
    xlabel('X [m]','FontSize',8); ylabel('TOI [-]','FontSize',8);
    grid on; box on;
end
sgtitle(sprintf('최적 구조 검증 — 법선력 & TOI 안정성  [Rocker:%s / Bogie:%s  f=%.4f]', ...
        p_opt.rocker_mode, p_opt.bogie_mode, f_opt), ...
        'FontSize',10, 'FontWeight','bold');

%% ── Figure 2: 모터 토크 (링크 관성 분해) ────────────────────
figure('Name','최적 구조 — 모터 토크 분해', ...
       'Position',[40 40 1500 700], 'Color','w');

for ti = 1:4
    rv  = results_v(ti);
    D   = rv.D; x = rv.x_arr;
    col = t_cols{ti};

    subplot(2,4,ti); hold on;
    plot(x, D.tau_max_arr*1000, '-', 'Color',col, 'LineWidth',2.0, ...
         'DisplayName','총 토크');
    area(x, (D.tau_rocker_inertia+D.tau_bogie_inertia)*1000, ...
         'FaceColor',col, 'FaceAlpha',0.22, 'EdgeColor','none', ...
         'DisplayName','링크 관성분');
    scatter(x(D.edge_mask), D.tau_max_arr(D.edge_mask)*1000, ...
            15, col, 'filled', 'HandleVisibility','off');
    yline(D.stair_torque_peak*1000, '--r', ...
          sprintf('95th=%.1fmNm', D.stair_torque_peak*1000), ...
          'FontSize',7, 'LineWidth',1.2);
    title({terrain_nm{ti}, ...
           sprintf('링크관성 %.1f%%  liftoff 뒷%d/앞%d', ...
           D.tau_link_ratio*100, sum(D.liftoff_r), sum(D.liftoff_f))}, ...
           'FontSize',8, 'FontWeight','bold');
    if ti==1, legend('FontSize',6,'Location','northwest'); end
    xlabel('X [m]','FontSize',8); ylabel('토크 [mNm]','FontSize',8);
    grid on; box on;

    subplot(2,4,ti+4); hold on;
    plot(x, D.alpha_rocker, '-', 'Color',[0.2 0.4 0.8], 'LineWidth',1.5, ...
         'DisplayName','\alpha_{Rocker}');
    plot(x, D.alpha_bogie,  '-', 'Color',[0.8 0.4 0.2], 'LineWidth',1.5, ...
         'DisplayName','\alpha_{Bogie}');
    yline(0, 'k--', 'LineWidth',0.5);
    title(sprintf('%s 링크 각가속도', terrain_nm{ti}), ...
          'FontSize',8, 'FontWeight','bold');
    if ti==1, legend('FontSize',6,'Location','best'); end
    xlabel('X [m]','FontSize',8); ylabel('rad/s²','FontSize',8);
    grid on; box on;
end
sgtitle('모터 토크 분해 — D6374 150KV 기준 (링크 관성 포함)', ...
        'FontSize',10, 'FontWeight','bold');

%% ── Figure 3: S/N비 이상 궤적 (v3 지형 기반) ────────────────
figure('Name','S/N비 이상 궤적 비교', ...
       'Position',[60 60 1500 700], 'Color','w');

for ti = 1:4
    rv  = results_v(ti);
    M   = rv.M; R = rv.R; x = rv.x_arr;
    x_t = rv.x_t; y_t = rv.y_t;
    col = t_cols{ti};

    x_valid = x;
    if isfield(R,'ok') && any(~R.ok)
        x_valid = x(R.ok);
    end
    % M.y0_valid / M.y_ideal : calc_metrics v3가 반환하는 유효 포인트 배열
    % 길이 안전 처리: 두 배열 중 짧은 쪽에 맞춤
    n_v       = min(numel(M.y0_valid), numel(x_valid));
    x_v       = x_valid(1:n_v);
    y0_v      = M.y0_valid(1:n_v);
    y_ideal_v = M.y_ideal(1:n_v);

    subplot(2,4,ti); hold on;
    plot(x_t, y_t, 'k-', 'LineWidth',1.2, 'HandleVisibility','off');
    plot(x_v, y0_v, '-',  'Color',col, 'LineWidth',2.0, 'DisplayName','차체 높이');
    plot(x_v, y_ideal_v,  '--', 'Color',[0.2 0.7 0.3], 'LineWidth',1.5, ...
         'DisplayName','이상 (y_t+R_w)');
    win = max(3, min(40, floor(n_v/4)));
    plot(x_v, movmean(y0_v, win), ':', 'Color',[0.7 0.3 0.8], ...
         'LineWidth',1.2, 'DisplayName','이상 v2 (movmean)');
    title({terrain_nm{ti}, ...
           sprintf('SN_v3=%.1fdB  SN_v2=%.1fdB', M.SN_dB, M.SN_dB_v2)}, ...
           'FontSize',8, 'FontWeight','bold');
    if ti==1, legend('FontSize',6,'Location','northwest'); end
    xlabel('X [m]','FontSize',8); ylabel('Y [m]','FontSize',8);
    grid on; box on; xlim([x_t(1), x_t(end)]);

    subplot(2,4,ti+4); hold on;
    err_v3    = y0_v - y_ideal_v;
    err_v3_dm = err_v3 - mean(err_v3, 'omitnan');
    err_v2    = y0_v - movmean(y0_v, win);
    plot(x_v, err_v3_dm*1000, '-', 'Color',[0.2 0.7 0.3], 'LineWidth',1.5, ...
         'DisplayName','오차 v3');
    plot(x_v, err_v2*1000,    '-', 'Color',[0.7 0.3 0.8], 'LineWidth',1.0, ...
         'DisplayName','오차 v2');
    yline(0,'k--','LineWidth',0.5);
    title(sprintf('%s  σ_v3=%.2fmm  σ_v2=%.2fmm', ...
          terrain_nm{ti}, std(err_v3_dm)*1000, std(err_v2)*1000), ...
          'FontSize',8, 'FontWeight','bold');
    if ti==1, legend('FontSize',6,'Location','best'); end
    xlabel('X [m]','FontSize',8); ylabel('오차 [mm]','FontSize',8);
    grid on; box on; xlim([x_t(1), x_t(end)]);
end
sgtitle('S/N비 이상 궤적 비교 — v3(지형기반) vs v2(movmean)', ...
        'FontSize',10, 'FontWeight','bold');

%% ── Figure 4: 로봇 형상 스케치 (실제 계단) ──────────────────
figure('Name','최적 구조 형상', 'Position',[80 80 900 520], 'Color','w');
rv_s = results_v(1);   % real_stairs
hold on;
fill([rv_s.x_t, fliplr(rv_s.x_t)], ...
     [rv_s.y_t, -0.06*ones(1,length(rv_s.y_t))], ...
     [0.78 0.68 0.58], 'EdgeColor','none', 'FaceAlpha',0.5);
plot(rv_s.x_t, rv_s.y_t, 'k-', 'LineWidth',1.2, 'HandleVisibility','off');

snap_idx = round(linspace(20, length(rv_s.x_arr)-20, 7));
th_w = linspace(0,2*pi,24);
hv   = {'HandleVisibility','off'};

for fi = snap_idx
    R_s = rv_s.R;
    y0_=R_s.y0(fi); ar_=R_s.ar(fi); bb_=R_s.bb(fi);
    [Wf_,Wm_,Wr_,Pb_] = wpos([y0_;ar_;bb_], rv_s.x_arr(fi), p_opt);
    P0_ = [rv_s.x_arr(fi); y0_];

    switch lower(p_opt.rocker_mode)
        case {'linear','triangle'}
            plot([Wr_(1),P0_(1),Pb_(1)],[Wr_(2),P0_(2),Pb_(2)], ...
                 'b-','LineWidth',2.5,hv{:});
        case 'frame'
            ar_e=ar_+p_opt.phi_r0;
            ur=[cos(ar_e);sin(ar_e)];
            Ptr=P0_-p_opt.j_r*p_opt.T_r*ur;
            Ptf=P0_+(1-p_opt.j_r)*p_opt.T_r*ur;
            plot([Ptr(1),Ptf(1)],[Ptr(2),Ptf(2)],'b-','LineWidth',2.5,hv{:});
            plot([Ptf(1),Pb_(1)],[Ptf(2),Pb_(2)],'b-','LineWidth',2,hv{:});
            plot([Ptr(1),Wr_(1)],[Ptr(2),Wr_(2)],'b-','LineWidth',2,hv{:});
    end
    switch lower(p_opt.bogie_mode)
        case {'linear','triangle'}
            plot([Wm_(1),Pb_(1),Wf_(1)],[Wm_(2),Pb_(2),Wf_(2)], ...
                 'g-','LineWidth',2.5,hv{:});
        case 'frame'
            ubb=[cos(bb_);sin(bb_)];
            Pbm=Pb_-p_opt.j_b*p_opt.T_b*ubb;
            Pbf=Pb_+(1-p_opt.j_b)*p_opt.T_b*ubb;
            plot([Pbm(1),Pbf(1)],[Pbm(2),Pbf(2)],'g-','LineWidth',2.5,hv{:});
            plot([Pbf(1),Wf_(1)],[Pbf(2),Wf_(2)],'g-','LineWidth',2,hv{:});
            plot([Pbm(1),Wm_(1)],[Pbm(2),Wm_(2)],'g-','LineWidth',2,hv{:});
    end
    ar_e=ar_+p_opt.phi_r0;
    ur=[cos(ar_e);sin(ar_e)]; nr=[-sin(ar_e);cos(ar_e)];
    bw=0.08;
    fill([P0_(1)-bw*ur(1),P0_(1)+bw*ur(1), ...
          P0_(1)+bw*ur(1)+p_opt.h_body*nr(1), ...
          P0_(1)-bw*ur(1)+p_opt.h_body*nr(1)], ...
         [P0_(2)-bw*ur(2),P0_(2)+bw*ur(2), ...
          P0_(2)+bw*ur(2)+p_opt.h_body*nr(2), ...
          P0_(2)-bw*ur(2)+p_opt.h_body*nr(2)], ...
         [0.4 0.6 0.85],'FaceAlpha',0.5,'EdgeColor',[0.1 0.3 0.7],hv{:});
    for Ww = {Wf_,Wm_,Wr_}
        w=Ww{1};
        fill(w(1)+p_opt.R_w*cos(th_w), w(2)+p_opt.R_w*sin(th_w), ...
             [0.2 0.2 0.2],'FaceAlpha',0.8,'EdgeColor','none',hv{:});
    end
end
plot(rv_s.x_arr, rv_s.R.y0, '-', 'Color',[0.20 0.55 0.35], ...
     'LineWidth',2.2, 'DisplayName','차체 궤적');
legend('Location','northwest','FontSize',8);
xlim([rv_s.x_t(1), rv_s.x_t(end)]);
ylim([-0.05, max(rv_s.y_t)+p_opt.R_w*4+p_opt.h_body+0.15]);
grid on; box on;
xlabel('X [m]','FontSize',10); ylabel('Y [m]','FontSize',10);
title({'최적 구조 — 실제 계단 주행', ...
       sprintf('Rocker:%s  Bogie:%s  f=%.4f', ...
               p_opt.rocker_mode, p_opt.bogie_mode, f_opt)}, ...
      'FontSize',10, 'FontWeight','bold');

%% ═════════════════════════════════════════════════════════════
%% [SECTION 12]  가중치 민감도 분석 (RUN_SENSITIVITY = true 시 실행)
%% ═════════════════════════════════════════════════════════════
if RUN_SENSITIVITY
    fprintf('\n%s\n', repmat('═',1,60));
    fprintf('  가중치 민감도 분석 시작\n');
    fprintf('  섭동 범위: ±10%%  | 지형 포인트: %d\n', N_PTS);
    fprintf('%s\n', repmat('─',1,60));

    % 섭동 대상 가중치 필드명
    w_fields = {'tau','imbal','stab','sn','fail'};
    n_fields = length(w_fields);

    % 결과 저장: [필드수 × 섭동수] 행렬
    sens_f_vals = zeros(n_fields, N_SENS_PERTURB);
    sens_grid   = linspace(-0.10, +0.10, N_SENS_PERTURB);  % ±10% 균등 격자

    for wi = 1:n_fields
        fname = w_fields{wi};
        w_nom = W.(fname);   % 명목 가중치

        for p_idx = 1:N_SENS_PERTURB   % [FIX-4] pi_s→p_idx (pi는 원주율 상수와 혼동)
            W_perturb = W;
            delta = w_nom * sens_grid(p_idx);
            W_perturb.(fname) = max(w_nom + delta, 0);

            % 나머지 가중치 합 정규화 (합이 항상 1 유지)
            other_fields = w_fields(~strcmp(w_fields, fname));
            w_sum_other  = 0;
            for oi = 1:length(other_fields)
                w_sum_other = w_sum_other + W.(other_fields{oi});
            end
            scale = (1 - W_perturb.(fname)) / max(w_sum_other, 1e-6);
            for oi = 1:length(other_fields)
                W_perturb.(other_fields{oi}) = W.(other_fields{oi}) * scale;
            end
            % [수정] objective 함수 인자에 P0_HEIGHT_MAX 추가
            f_perturb = objective(x_opt, p0, N_PTS, WBOT_MIN, WBOT_MAX, ...
                                  FAIL_MAX, LIFTOFF_MAX, TOI_WARN, ...
                                  TAU_REF, IMBAL_REF, SN_REF, W_perturb, W_terrain, P0_HEIGHT_MAX);
            sens_f_vals(wi, p_idx) = f_perturb;
        end

        % 민감도 = df/dw 근사 (중앙 차분)
        df_dw = (sens_f_vals(wi,end) - sens_f_vals(wi,1)) / (0.20 * w_nom + 1e-9);
        fprintf('  W.%-6s  명목=%.2f  df/dw=%.4f  f 범위=[%.4f, %.4f]\n', ...
                fname, w_nom, df_dw, ...
                min(sens_f_vals(wi,:)), max(sens_f_vals(wi,:)));
    end

    % 민감도 시각화
    figure('Name','가중치 민감도 분석', ...
           'Position',[100 100 1000 500], 'Color','w');
    w_labels = {'W_{τ}','W_{imbal}','W_{stab}','W_{SN}','W_{fail}'};
    w_colors = {[0.7 0.2 0.2],[0.2 0.5 0.8],[0.2 0.7 0.4], ...
                [0.8 0.6 0.1],[0.5 0.3 0.7]};

    for wi = 1:n_fields
        subplot(1, n_fields, wi); hold on;
        w_nom  = W.(w_fields{wi});
        x_axis = w_nom * (1 + sens_grid);
        plot(x_axis, sens_f_vals(wi,:), '-o', ...
             'Color',w_colors{wi}, 'LineWidth',2.0, 'MarkerSize',5, ...
             'MarkerFaceColor',w_colors{wi});
        xline(w_nom, 'k--', '명목', 'FontSize',8);
        xlabel(w_labels{wi}, 'FontSize',9);
        ylabel('목적함수 f', 'FontSize',8);
        title(sprintf('민감도: %s', w_labels{wi}), 'FontSize',9, 'FontWeight','bold');
        grid on; box on;
    end
    sgtitle('가중치 민감도 분석 — 현재 최적해 기준 ±10% 섭동', ...
            'FontSize',10, 'FontWeight','bold');

    fprintf('%s\n', repmat('═',1,60));
    fprintf('민감도 분석 완료.\n\n');
end

fprintf('=== 완료 ===\n');
fprintf('저장: %s\n', save_path);