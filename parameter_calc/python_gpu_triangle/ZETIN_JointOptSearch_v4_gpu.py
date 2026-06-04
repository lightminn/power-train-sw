"""
ZETIN_JointOptSearch_v4_gpu.py
Rocker x Bogie 전체 파라미터 최적 탐색 — 삼각형/사각형 동시 탐색 버전

v3 대비 변경사항:
  - x[0] (rocker_mode): 1=triangle, 2=frame  (기존: 2로 고정 → 이제 자유 탐색)
  - x[1] (bogie_mode):  1=triangle, 2=frame  (기존: 2로 고정 → 이제 자유 탐색)
  - x[14] (brk_v): 브래킷 피벗 → 휠 축 수직 오프셋 (m). 탐색 범위 0.345~0.375.
                   링크 끝점 = 브래킷 피벗으로 해석되며, 휠 축은 그 아래 brk_v.
  - 4가지 모드 조합 (frame-frame, frame-tri, tri-frame, tri-tri) 동시 탐색
  - JAX lax.switch가 모든 모드 분기를 단일 컴파일로 처리

가속 전략 (v3과 동일):
  1. kin_sim: JAX vmap + 배치 Newton 솔버 (GPU 병렬 160포인트 동시)
  2. calc_dynamics: numpy 벡터 연산
  3. calc_stability: 완전 벡터화
  4. 최적화: scipy differential_evolution
"""
import os
import sys
import time
import pickle
import numpy as np

# JAX GPU 설정 (반드시 import 전에)
os.environ.setdefault('JAX_PLATFORM_NAME', 'gpu')
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE', 'false')

import jax
import jax.numpy as jnp

print(f'JAX devices: {jax.devices()}')
print(f'JAX default backend: {jax.default_backend()}')

# python_gpu_triangle/functions/ 로컬 사용 (v4 전용 fork — v3 동결 유지).
# v3 결과(zetin_optimal_params_v3.pkl) 재현은 ../python_gpu/ 디렉토리에서 수행할 것.
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from functions.gen_terrain import gen_terrain
from functions.calc_envelope_jax import calc_envelope_gpu
from functions.newton_solver import kin_sim_gpu
from functions.calc_dynamics_jax import calc_dynamics_gpu
from functions.calc_stability_jax import calc_stability_gpu
from functions.calc_metrics_jax import calc_metrics_gpu
from functions.wpos_jax import pack_params_auto, wpos_jax

from scipy.optimize import differential_evolution

# ═══════════════════════════════════════
np.random.seed(2026)

# ═══════════════════════════════════════
# [SECTION 1] 공통 파라미터 (v3과 동일)
# ═══════════════════════════════════════
# Phase 2 모터 변경: D6374+외부5:1 → BL70200 8" 인휠 BLDC (내장 1:5 hub motor).
# 48V 시스템: 휠 측 정격 22Nm / 피크 39Nm / 무부하 240 RPM (= 25.1 rad/s = 2.51 m/s linear).
# 0.5 km/h(0.14 m/s) 이하 지속운전 부적합. m_wheel 4.5kg = 모터 포함 휠 단위.
p0 = {
    'R_w': 0.100, 'h_body': 0.300, 'mass': 50, 'g': 9.81,
    'obs_h': 0.150, 'mu': 0.70,
    # 인휠 모터: 외부 기어 없음 (내장 1:5는 사양 토크값에 이미 포함).
    'gear_ratio': 1.0, 'eta_gear': 1.0,
    'motor_tau_peak': 39.0,    # 휠 측 피크 (Nm)
    'motor_tau_cont': 22.0,    # 휠 측 정격 (Nm)
    'omega_no_load_rpm': 240.0,# 휠 측 무부하 RPM @ 48V (= 25.1 rad/s)
    'V_bus': 48.0,             # 배터리 전압
    'v_min_advisable': 0.14,   # 0.5 km/h, 이하 지속운전 부적합
    # Phase 3+ (B-2): 배터리 전류 한계. 모터 정격 9A × 6 = 54A 이론치, 보수적 30A.
    'Kt_eff': 22.0 / 9.0,      # 휠 측 τ/I (Nm/A) — BL70200 정격 22Nm @ 9A
    'battery_max_current': 30.0,
    'n_wheel_total': 6,
    'm_wheel': 4.5,            # 휠+모터 일체 단위 (이전 3.5kg 휠만)
    'm_rocker_link': 2.5,      # 본체 50kg에 맞춰 링크 질량도 상향 (이전 1.5)
    'm_bogie_link': 1.5,       # 이전 0.8
    # 링크 외 추가 관성 (브래킷, 케이블, 마운트 등 추정 — 실측 시 갱신).
    'I_rocker_add': 0.15, 'I_bogie_add': 0.08,
    'e_restitution': 0.3,
    # Phase 1: 사다리꼴 속도 프로파일 — 가속/감속 한계 명시.
    'v_robot': 0.8, 'v_max': 0.8, 'a_lim': 1.5, 'v_max_flat': 2.0,
    'step_thresh': 5.0,
    'phi_r0': 0, 'delta_pb': 0,
    # Phase 1: 비대칭 CG — 배터리/모터/탑재물이 전방으로 50mm 편위 가정.
    # h_body=300mm, mass=50kg: 본체 중심 0.18m 위 가정.
    'CG_offset': 0.050,
    # Phase 3+ Tier C-2: 휠 접촉 패치 폭 (m). BL70200 8" 70mm 폭 타이어 기준.
    # 단단한 노면 ~3cm, 부드러운 노면 ~5cm. 보수적 3cm.
    'patch_width': 0.030,
}
p0['h_CG'] = p0['h_body'] * 0.55

# ═══════════════════════════════════════
# [SECTION 2] 목적함수 설계 상수 (v3과 동일)
# ═══════════════════════════════════════
TAU_REF = 15.0   # 휠 측 Nm — 신규 모터(BL70200) 정격 22Nm의 ~68% 기준. (이전 1.85 모터측)
IMBAL_REF = 10
SN_REF = 35
WBOT_MIN = 0.400
WBOT_MAX = 0.700
FAIL_MAX = 0.10
LIFTOFF_MAX = 0.02
TOI_WARN = 0.20
P0_HEIGHT_MAX = 0.900  # brk_v(≤0.375m) 추가로 P0 평지높이 상한 확대
# Phase 2 인휠 모터 변경: 휠 측 직접 토크 기준. gear_ratio=1, eta=1이므로 휠=모터.
TAU_MOTOR_SAT = p0['motor_tau_peak']  # 39 Nm (BL70200 휠 측 피크)

# v4 Phase 3+ 재분배 — 'cont'(연속토크), 'batt'(배터리), 'stuck'(시스템견인) 추가.
# 합 1.0 유지. 에너지는 보고용 (objective 미포함).
W = {'tau': 0.12, 'imbal': 0.08, 'stab': 0.18, 'sn': 0.06, 'fail': 0.10,
     'sat': 0.12, 'slip': 0.12,
     'cont': 0.10,   # 연속 정격 토크 (열적 한계)
     'batt': 0.06,   # 배터리 전류 한계
     'stuck': 0.06}  # 시스템 견인력 부족
# v4 Phase 2 확장: 경사 슬로프 2종 추가, 기존 항목에서 5%씩 가져와 10% 할당.
W_terrain = {'stairs': 0.45, 'wood': 0.12, 'rough': 0.13, 'step': 0.10,
             'curved_ramp': 0.10, 'incline_15': 0.05, 'incline_30': 0.05}
# v4 Phase 2c+2d: 지형별 정상 마찰계수 — 실제 표면 특성 반영.
MU_TERRAIN = {
    'flat': 0.70, 'step': 0.65, 'stairs': 0.60, 'real_stairs': 0.60,
    'wood_block': 0.70, 'rough': 0.55, 'curved_ramp': 0.65,
    'incline_15': 0.65, 'incline_30': 0.60,  # 경사 표면 (다소 보수적)
}
N_PTS = int(os.environ.get('N_PTS', 100))  # Phase 3+ 가속: 160→100 (envelope는 dense 8000-grid 그대로, 평가 포인트만 축소)
# Phase 3+ Tier C-1: 적응적 샘플링 — 단차 모서리 근처 밀도 증가.
EDGE_BOOST = 3.0  # 1.0=균등, 3.0이면 edge 4x 밀도


def adaptive_xa(x_t, y_t_env, xs, xe, n_pts=N_PTS, edge_boost=EDGE_BOOST):
    """terrain gradient 따라 샘플 밀도 biased — 인버스 CDF 방식.

    JAX shape 안정성을 위해 n_pts는 고정. 분포만 변경.

    Args:
        x_t: terrain x 그리드 (dense, N≈8000)
        y_t_env: 팽창 envelope y
        xs, xe: 샘플 구간 [xs, xe]
        n_pts: 출력 샘플 수 (고정)
        edge_boost: 그라디언트 피크 부근 밀도 배율 (1.0=균등)

    Returns:
        xa: n_pts 위치 배열, 단차 부근 더 조밀
    """
    mask = (x_t >= xs) & (x_t <= xe)
    if np.sum(mask) < 10:
        return np.linspace(xs, xe, n_pts)
    xt_sub = x_t[mask]
    yt_sub = y_t_env[mask]

    grad = np.abs(np.gradient(yt_sub, xt_sub))
    grad_max = grad.max()
    if grad_max < 1e-3:  # 평지 — 균등 분포
        return np.linspace(xs, xe, n_pts)

    grad_norm = grad / (grad_max + 1e-9)
    density = 1.0 + edge_boost * grad_norm  # ∈ [1, 1+edge_boost]

    # CDF
    cdf = np.cumsum(density)
    cdf = cdf / cdf[-1]
    # 인버스 CDF: 균등 u → density-warped x
    u = np.linspace(0.0, 1.0, n_pts)
    xa = np.interp(u, cdf, xt_sub)
    return xa.astype(np.float64)


print('=== ZETIN GPU 가속 최적화 v4 (삼각형+사각형 동시 탐색) ===')
print(f'JAX backend: {jax.default_backend()}')
print(f'스펙: R_w={p0["R_w"]*1000:.0f}mm  mass={p0["mass"]:.0f}kg')
print(f'TAU_REF={TAU_REF:.2f}Nm  WBOT=[{WBOT_MIN*1000:.0f}~{WBOT_MAX*1000:.0f}]mm')
print(f'탐색 모드: Rocker={{triangle, frame}}  Bogie={{triangle, frame}}\n')


# ═══════════════════════════════════════
# [SECTION 5] 헬퍼 함수 (v3과 동일)
# ═══════════════════════════════════════

def decode_x(x, p0):
    p = dict(p0)
    rm = int(round(x[0]))
    bm = int(round(x[1]))

    if bm == 1:
        p['bogie_mode'] = 'triangle'
        p['L_b1'] = x[8]; p['L_b2'] = x[9]
        p['beta_b'] = np.deg2rad(x[10])
        p['c_b'] = p['L_b1'] * abs(np.sin(p['beta_b'] / 2))
        p['d_b'] = p['L_b2'] * abs(np.sin(p['beta_b'] / 2))
        h_bogie_drop = p['L_b1'] * np.cos(-np.pi / 2 + p['beta_b'] / 2)
    elif bm == 2:
        p['bogie_mode'] = 'frame'
        p['T_b'] = x[8]; p['S_b1'] = x[9]; p['S_b2'] = x[10] * 5e-3
        p['th_b1'] = np.deg2rad(x[11]); p['th_b2'] = np.deg2rad(x[12]); p['j_b'] = x[13]
        N_bb = p['S_b1'] * np.cos(p['th_b1']) - p['S_b2'] * np.cos(p['th_b2'])
        D_bb = p['T_b'] + p['S_b1'] * np.sin(p['th_b1']) + p['S_b2'] * np.sin(p['th_b2'])
        bb_0 = np.arctan2(N_bb, D_bb)
        h_bogie_drop = -(1 - p['j_b']) * p['T_b'] * np.sin(bb_0) + p['S_b1'] * np.cos(p['th_b1'] + bb_0)
        Wf_c = abs((1 - p['j_b']) * p['T_b'] * np.cos(bb_0) + p['S_b1'] * np.sin(p['th_b1'] + bb_0))
        Wm_c = abs(p['j_b'] * p['T_b'] * np.cos(bb_0) + p['S_b2'] * np.sin(p['th_b2'] - bb_0))
        p['c_b'] = max(Wf_c, p0['R_w']); p['d_b'] = max(Wm_c, p0['R_w'])
    else:
        return None

    if rm == 1:
        p['rocker_mode'] = 'triangle'
        p['L_r1'] = x[2]; p['L_r2'] = x[3]; p['alpha_r'] = np.deg2rad(x[4])
    elif rm == 2:
        p['rocker_mode'] = 'frame'
        p['T_r'] = x[2]; p['S_r1'] = x[3]
        p['th_r1'] = np.deg2rad(x[5]); p['th_r2'] = np.deg2rad(x[6]); p['j_r'] = x[7]
        h_rocker_front = p['S_r1'] * np.cos(p['th_r1'])
        p['S_r2'] = (h_rocker_front + h_bogie_drop) / np.cos(p['th_r2'])
    else:
        return None

    p['brk_v'] = x[14]
    return p


def calc_wbot(p):
    mode = p.get('bogie_mode', 'linear').lower()
    if mode == 'triangle':
        return (p['L_b1'] + p['L_b2']) * np.sin(p['beta_b'] / 2)
    elif mode == 'frame':
        return max(p['S_b1'] * abs(np.cos(p['th_b1'])) + p['S_b2'] * abs(np.cos(p['th_b2'])),
                   (p['S_b1'] + p['S_b2']) * 0.7)
    return p['c_b'] + p['d_b']


def get_cb_fwd(p):
    mode = p.get('bogie_mode', 'linear').lower()
    if mode == 'triangle': return max(p['L_b1'] * abs(np.sin(p['beta_b'] / 2)), p['R_w'])
    elif mode == 'frame': return max(p['S_b1'] * abs(np.cos(p['th_b1'])), p['R_w'])
    return max(p.get('c_b', 0.14), p['R_w'])


def get_b_eff(p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'triangle': return max(p['L_r2'] * abs(np.cos(p['alpha_r'] / 2)), p['R_w'])
    elif mode == 'frame': return max(p['j_r'] * p['T_r'] + p['S_r2'] * abs(np.sin(p['th_r2'])), p['R_w'])
    return max(p.get('b_r', 0.28), p['R_w'])


def get_a_eff(p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'triangle': return max(p['L_r1'] * abs(np.cos(p['alpha_r'] / 2)), p['R_w'])
    elif mode == 'frame': return max((1 - p['j_r']) * p['T_r'] + p['S_r1'] * abs(np.sin(p['th_r1'])), p['R_w'])
    return max(p.get('a_r', 0.22), p['R_w'])


def calc_P0_height_flat(p):
    # 링크 끝점(브래킷 피벗)이 평지 위 R_w + brk_v 높이에 위치하고, P0는 그 위로 링크 만큼 더 올라감.
    base = p['R_w'] + p.get('brk_v', 0.0)
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'triangle': return max(base + p['L_r2'] * np.sin(p['alpha_r'] / 2), base)
    elif mode == 'frame': return max(base + p['S_r2'] * np.cos(p['th_r2']), base)
    return base


# ═══════════════════════════════════════
# JIT 워밍업 — 4가지 모드 조합 모두 컴파일
# lax.switch는 단일 컴파일로 모든 분기를 처리하지만
# newton_solver의 초기값 계산 경로 확인을 위해 전체 워밍업 수행
# ═══════════════════════════════════════
def _make_warmup_p(rocker_mode, bogie_mode):
    wp = dict(p0)
    if rocker_mode == 'triangle':
        wp['rocker_mode'] = 'triangle'
        wp['L_r1'] = 0.30; wp['L_r2'] = 0.25; wp['alpha_r'] = np.deg2rad(120)
    else:
        wp['rocker_mode'] = 'frame'
        wp['T_r'] = 0.3; wp['S_r1'] = 0.2; wp['S_r2'] = 0.15
        wp['th_r1'] = 0.3; wp['th_r2'] = 0.3; wp['j_r'] = 0.5
    if bogie_mode == 'triangle':
        wp['bogie_mode'] = 'triangle'
        wp['L_b1'] = 0.22; wp['L_b2'] = 0.20; wp['beta_b'] = np.deg2rad(120)
        wp['c_b'] = wp['L_b1'] * abs(np.sin(wp['beta_b'] / 2))
        wp['d_b'] = wp['L_b2'] * abs(np.sin(wp['beta_b'] / 2))
    else:
        wp['bogie_mode'] = 'frame'
        wp['T_b'] = 0.2; wp['S_b1'] = 0.18; wp['S_b2'] = 0.12
        wp['th_b1'] = 0.3; wp['th_b2'] = 0.3; wp['j_b'] = 0.5
        wp['c_b'] = 0.14; wp['d_b'] = 0.14
    return wp


print('JAX JIT 워밍업 중 (4가지 모드 조합 컴파일, 2-3분 소요)...')
_t_warmup = time.time()

_mode_combos = [
    ('frame', 'frame'),
    ('frame', 'triangle'),
    ('triangle', 'frame'),
    ('triangle', 'triangle'),
]

for _rm, _bm in _mode_combos:
    _wp = _make_warmup_p(_rm, _bm)
    _wp_arr = pack_params_auto(_wp)
    _flat_x, _flat_y = gen_terrain('flat', _wp)
    _flat_env = calc_envelope_gpu(_flat_x, _flat_y, _wp['R_w'], patch_width=_wp.get('patch_width', 0.030))
    _warmup_xa = np.linspace(_flat_x[0] + 0.3, _flat_x[-1] - 0.3, N_PTS)
    _R_w = kin_sim_gpu(_warmup_xa, _flat_x, _flat_env, _wp_arr, _wp)
    print(f'  {_rm}-{_bm}: 실패율 {_R_w["fail_rate"]*100:.1f}%')

print(f'JAX 워밍업 완료! ({time.time()-_t_warmup:.1f}초)\n')


# ═══════════════════════════════════════
# 지형 / Envelope 사전 계산 캐시
# 지형(gen_terrain)은 p0['obs_h']에만, envelope(calc_envelope_gpu)는 p0['R_w']·patch_width에만
# 의존한다 — 이 셋은 모두 탐색 변수와 무관한 상수다(decode_x는 기하 파라미터만 바꾼다).
# 따라서 objective 매 호출마다 재계산하던 8지형 × 8000점 O(N²) envelope를 1회만 계산해 캐시한다.
# 수만 번의 DE 평가에서 큰 절감이며, 결과는 완전히 동일하다.
# ═══════════════════════════════════════
_ALL_TERRAINS = ['flat', 'real_stairs', 'wood_block', 'rough', 'step',
                 'curved_ramp', 'incline_15', 'incline_30']


def _build_terrain_cache(p_ref):
    cache = {}
    pw = p_ref.get('patch_width', 0.030)
    for _t in _ALL_TERRAINS:
        _xt, _yt_raw = gen_terrain(_t, p_ref)
        _yt_env = calc_envelope_gpu(_xt, _yt_raw, p_ref['R_w'], patch_width=pw)
        cache[_t] = (_xt, _yt_raw, _yt_env)
    return cache


print('지형/Envelope 캐시 생성 중 (8지형, 1회)...')
_t_cache = time.time()
TERRAIN_CACHE = _build_terrain_cache(p0)
print(f'지형 캐시 완료! ({time.time()-_t_cache:.1f}초)\n')


# ═══════════════════════════════════════
# [SECTION 4] 목적함수 (v3과 동일, 모드 변수화)
# ═══════════════════════════════════════

def objective(x):
    x = list(x)
    x[0] = round(x[0]); x[1] = round(x[1])

    p = decode_x(x, p0)
    if p is None: return 10

    Wbot = calc_wbot(p)
    if Wbot < WBOT_MIN or Wbot > WBOT_MAX:
        return 50 + abs(Wbot - (WBOT_MIN + WBOT_MAX) / 2)

    y0_flat = calc_P0_height_flat(p)
    if y0_flat > P0_HEIGHT_MAX:
        return 50 + (y0_flat - P0_HEIGHT_MAX) * 10

    try:
        dist_mr = get_a_eff(p) + get_b_eff(p) - p.get('d_b', 0)
    except KeyError:
        return 10
    if dist_mr < 0.250:
        return 50 + abs(0.250 - dist_mr) * 10

    p_arr = pack_params_auto(p)

    # 평지 테스트 (GPU) — 캐시된 지형/envelope 사용 (상수 의존이므로 1회 계산분 재사용)
    try:
        flat_x, flat_y_raw, flat_y_env = TERRAIN_CACHE['flat']
        test_R = kin_sim_gpu(np.array([0.0]), flat_x, flat_y_env, p_arr, p)
        if not test_R['ok'][0]:
            return 60
    except Exception:
        return 60

    # 7종 지형 평가 (GPU 가속) — v4에서 curved_ramp, incline_15/30 추가
    terrains = ['real_stairs', 'wood_block', 'rough', 'step', 'curved_ramp',
                'incline_15', 'incline_30']
    t_weights = np.array([W_terrain['stairs'], W_terrain['wood'], W_terrain['rough'],
                          W_terrain['step'], W_terrain['curved_ramp'],
                          W_terrain['incline_15'], W_terrain['incline_30']])
    n_terr = len(terrains)

    tau_vals = np.zeros(n_terr)
    tau_peak_motor_all = np.zeros(n_terr)  # Phase 1: 절대 피크 모터 토크 (포화 판정용)
    sat_speed_aware_all = np.zeros(n_terr) # Phase 2d: 속도 인식 포화율 (peak)
    sat_viol_all = np.zeros(n_terr)        # Phase 2d: τ>τ_avail 포인트 비율
    sn_vals = np.zeros(n_terr)
    imbal_vals = np.zeros(n_terr)
    fail_pts = np.zeros(n_terr)
    total_pts = np.zeros(n_terr)
    toi_min_all = np.ones(n_terr)
    liftoff_all = np.zeros(n_terr)
    slip_viol_all = np.zeros(n_terr)  # Phase 2c: 슬립 위반율
    slip_peak_all = np.zeros(n_terr)  # Phase 2c: 슬립 피크
    # Phase 3+
    tau_rms_all = np.zeros(n_terr)        # A-1: 휠 RMS 토크 worst
    cont_viol_all = np.zeros(n_terr)      # A-1: 연속 한계 초과율
    stuck_rate_all = np.zeros(n_terr)     # B-1: 시스템 견인 부족
    batt_peak_all = np.zeros(n_terr)      # B-2: 배터리 전류 피크
    batt_viol_all = np.zeros(n_terr)      # B-2: 배터리 전류 초과율
    energy_Wh_all = np.zeros(n_terr)      # A-2: 지형당 에너지 (보고용)

    p['liftoff_max'] = LIFTOFF_MAX

    for ti, t in enumerate(terrains):
        try:
            x_t, y_t_raw, y_t_env = TERRAIN_CACHE[t]  # 캐시 재사용 (상수 의존)

            b_eff_v = get_b_eff(p); a_eff_v = get_a_eff(p); cb = get_cb_fwd(p)
            xs = x_t[0] + b_eff_v + 0.05
            xe = x_t[-1] - (a_eff_v + cb) - 0.05
            if xs >= xe: return 12
            # Phase 3+ C-1: 적응적 샘플링 — 단차 모서리 밀도 ↑
            xa = adaptive_xa(x_t, y_t_env, xs, xe, n_pts=N_PTS, edge_boost=EDGE_BOOST)

            R = kin_sim_gpu(xa, x_t, y_t_env, p_arr, p)
            fail_pts[ti] = np.sum(~R['ok'])
            total_pts[ti] = len(xa)

            # Phase 2c: 지형별 μ 적용 (calc_dynamics에서 p['mu']로 읽힘)
            p_t = dict(p)
            p_t['mu'] = MU_TERRAIN.get(t, p.get('mu', 0.65))

            D = calc_dynamics_gpu(R, xa, x_t, y_t_env, p_t)
            tau_vals[ti] = D['stair_torque_peak']
            tau_peak_motor_all[ti] = D['stair_torque_max']        # Phase 1: 절대 피크
            sat_speed_aware_all[ti] = D['sat_peak_speed_aware']   # Phase 2d
            sat_viol_all[ti] = D['sat_violation_rate']            # Phase 2d
            slip_viol_all[ti] = D['slip_violation_rate']          # Phase 2c
            slip_peak_all[ti] = D['slip_peak']                    # Phase 2c
            # Phase 3+
            tau_rms_all[ti] = D['tau_rms_worst']
            cont_viol_all[ti] = D['cont_violation_rate']
            stuck_rate_all[ti] = D['system_stuck_rate']
            batt_peak_all[ti] = D['battery_current_peak']
            batt_viol_all[ti] = D['battery_violation_rate']
            energy_Wh_all[ti] = D['energy_Wh']

            S = calc_stability_gpu(R, xa, x_t, y_t_raw, y_t_env, p_t)
            toi_min_all[ti] = S['min_TOI']
            liftoff_all[ti] = S['liftoff_ratio']

            if S.get('is_collision', False):
                return 9 + abs(S['min_clearance']) * 20
            if S['liftoff_ratio'] > LIFTOFF_MAX:
                return 7 + S['liftoff_ratio'] * 3

            valid_d = ~D['liftoff_r'] & ~D['liftoff_f'] & R['ok']
            if np.sum(valid_d) > 5:
                nm_ = [np.mean(D['Nr'][valid_d]), np.mean(D['Nm'][valid_d]), np.mean(D['Nf'][valid_d])]
            else:
                nm_ = [np.mean(D['Nr']), np.mean(D['Nm']), np.mean(D['Nf'])]
            if np.mean(nm_) > 0.5:
                imbal_vals[ti] = (max(nm_) - min(nm_)) / np.mean(nm_) * 100

            M = calc_metrics_gpu(xa, x_t, y_t_env, R, p)
            sn_vals[ti] = M['SN_dB']

        except Exception as ME:
            if '수렴 실패' not in str(ME):
                print(f'  [예외] {t}: {ME}')
            return 10

    fail_rate_total = np.sum(fail_pts) / max(np.sum(total_pts), 1)
    if fail_rate_total > FAIL_MAX:
        return 5 + fail_rate_total * 5

    tau_norm = np.sum(t_weights * tau_vals) / TAU_REF
    imbal_norm = np.sum(t_weights * imbal_vals) / IMBAL_REF

    global_toi_min = np.min(toi_min_all)
    global_liftoff = np.max(liftoff_all)

    if global_toi_min >= TOI_WARN:
        toi_penalty = (0.5 - global_toi_min) * 2
    else:
        toi_penalty = 0.6 + ((TOI_WARN - global_toi_min) / TOI_WARN) * 5

    stab_penalty = max(toi_penalty, (global_liftoff / LIFTOFF_MAX) * 3)
    sn_norm = 1 / (1 + max(np.sum(t_weights * sn_vals), 0) / SN_REF)
    fail_norm = fail_rate_total * 10

    # Phase 1+2d: 모터 포화 페널티 — 속도 인식 τ_avail 기준.
    # Phase 1은 고정 τ_peak=4.95 사용했으나, Phase 2d는 ω(v)에 따른 가용 토크와 비교.
    # 가중평균 + 절대 worst를 모두 반영. 위반 비율도 반영.
    sat_peak_weighted = float(np.sum(t_weights * sat_speed_aware_all))
    sat_peak_worst = float(np.max(sat_speed_aware_all))
    sat_ratio_weighted = max(0.0, sat_peak_weighted - 1.0)  # 1.0 = τ_avail 한계
    sat_ratio_worst = max(0.0, sat_peak_worst - 1.0)
    sat_viol_weighted = float(np.sum(t_weights * sat_viol_all))
    sat_norm = min(2.0 * sat_ratio_weighted + 1.0 * sat_ratio_worst + 3.0 * sat_viol_weighted, 3.0)

    # Phase 2c: 슬립 페널티 — 위반율(slip>1)의 가중 합 + 피크 over-1 비례.
    slip_viol_weighted = float(np.sum(t_weights * slip_viol_all))
    slip_peak_worst = float(np.max(slip_peak_all))
    slip_overshoot = max(0.0, slip_peak_worst - 1.0)
    slip_norm = min(slip_viol_weighted * 10.0 + slip_overshoot * 0.5, 3.0)

    # Phase 3+ A-1: 연속 토크 페널티 (RMS vs 22Nm 정격).
    tau_rms_worst_global = float(np.max(tau_rms_all))
    cont_violation_weighted = float(np.sum(t_weights * cont_viol_all))
    cont_overshoot = max(0.0, tau_rms_worst_global / p0['motor_tau_cont'] - 1.0)
    cont_norm = min(2.0 * cont_overshoot + 3.0 * cont_violation_weighted, 3.0)

    # Phase 3+ B-1: 시스템 견인력 부족 (stuck) 페널티.
    stuck_weighted = float(np.sum(t_weights * stuck_rate_all))
    stuck_norm = min(stuck_weighted * 10.0, 3.0)

    # Phase 3+ B-2: 배터리 전류 한계 페널티.
    batt_peak_global = float(np.max(batt_peak_all))
    batt_viol_weighted = float(np.sum(t_weights * batt_viol_all))
    batt_overshoot = max(0.0, batt_peak_global / p0['battery_max_current'] - 1.0)
    batt_norm = min(1.5 * batt_overshoot + 3.0 * batt_viol_weighted, 3.0)

    f = (W['tau'] * tau_norm + W['imbal'] * imbal_norm
         + W['stab'] * stab_penalty + W['sn'] * sn_norm
         + W['fail'] * fail_norm + W['sat'] * sat_norm
         + W['slip'] * slip_norm
         + W['cont'] * cont_norm
         + W['stuck'] * stuck_norm
         + W['batt'] * batt_norm)
    return max(f, 0)


# ═══════════════════════════════════════
# [SECTION 3] 탐색 공간
# v4 변경사항: x[0], x[1] 범위를 [1,2]로 개방, x[14]에 brk_v 추가
#
# x[0] = rocker_mode: 1=triangle, 2=frame
# x[1] = bogie_mode:  1=triangle, 2=frame
#
# 파라미터 슬롯 (모드 무관 공유):
#   x[2:5]  → triangle rocker: L_r1, L_r2, alpha_r(deg)
#           → frame rocker:    T_r,   S_r1, (unused)
#   x[5:8]  → triangle rocker: (unused)
#           → frame rocker:    th_r1(deg), th_r2(deg), j_r
#   x[8:11] → triangle bogie:  L_b1, L_b2, beta_b(deg)
#           → frame bogie:     T_b,  S_b1, S_b2×5mm
#   x[11:14]→ triangle bogie:  (unused)
#           → frame bogie:     th_b1(deg), th_b2(deg), j_b
#   x[14]   → brk_v (m): 브래킷 피벗 → 휠 축 수직 오프셋
# ═══════════════════════════════════════
lb = [1, 1, 0.20, 0.15, 60,  0,  0,  0.30, 0.15, 0.15, 60,  0,  0,  0.30, 0.345]
ub = [2, 2, 0.45, 0.35, 160, 35, 35, 0.70, 0.35, 0.35, 160, 40, 40, 0.70, 0.375]
bounds = list(zip(lb, ub))

# ═══════════════════════════════════════
# [SECTION 7] 최적화 실행
# ═══════════════════════════════════════
# 환경 변수로 단축 실행(스모크 테스트) 가능.
#   DE_MAXITER=200 DE_POPSIZE=15 python ZETIN_JointOptSearch_v4_gpu.py
de_maxiter = int(os.environ.get('DE_MAXITER', 2000))
de_popsize = int(os.environ.get('DE_POPSIZE', 30))
# Phase 3+ 가속: 멀티프로세스 워커. DE 평가는 독립적이라 CPU 코어 병렬 가능.
# 단, GPU JAX는 multiprocess.fork에 안전하지 않음 — workers > 1 시 자동으로 CPU JAX 강제 권장.
de_workers = int(os.environ.get('DE_WORKERS', 1))
de_tol = float(os.environ.get('DE_TOL', 1e-4))

print(f'differential_evolution 실행 중 (삼각형+사각형 동시 탐색, maxiter={de_maxiter}, popsize={de_popsize}, workers={de_workers}, tol={de_tol:.0e}, N_PTS={N_PTS})...\n')
tic = time.time()

result = differential_evolution(
    objective, bounds,
    maxiter=de_maxiter, popsize=de_popsize, tol=de_tol, seed=2026,
    disp=True, workers=de_workers,
    polish=False,  # x[0], x[1]이 이산 파라미터이므로 L-BFGS-B 폴리싱 비활성화
)

x_opt = result.x
f_opt = result.fun
elapsed = time.time() - tic

print(f'\n[최적화 완료] 평가: {result.nfev}  f_opt: {f_opt:.4f}')
print(f'총 소요 시간: {elapsed/60:.1f}분\n')

# ═══════════════════════════════════════
# [SECTION 8] 결과 출력
# ═══════════════════════════════════════
x_opt[0] = round(x_opt[0]); x_opt[1] = round(x_opt[1])
p_opt = decode_x(x_opt, p0)

print('=' * 60)
print('  최적 구조 (v4: 삼각형/사각형 동시 탐색)')
print('-' * 60)
print(f'Rocker mode : {p_opt["rocker_mode"]}')
if p_opt['rocker_mode'] == 'triangle':
    print(f'  L_r1={p_opt["L_r1"]*1000:.1f}mm  L_r2={p_opt["L_r2"]*1000:.1f}mm  '
          f'alpha_r={np.rad2deg(p_opt["alpha_r"]):.1f}deg')
elif p_opt['rocker_mode'] == 'frame':
    print(f'  T_r={p_opt["T_r"]*1000:.1f}mm  S_r1={p_opt["S_r1"]*1000:.1f}mm  S_r2={p_opt["S_r2"]*1000:.1f}mm')
    print(f'  th_r1={np.rad2deg(p_opt["th_r1"]):.1f}deg  th_r2={np.rad2deg(p_opt["th_r2"]):.1f}deg  j_r={p_opt["j_r"]:.2f}')

print(f'Bogie mode  : {p_opt["bogie_mode"]}')
if p_opt['bogie_mode'] == 'triangle':
    Wb = (p_opt['L_b1'] + p_opt['L_b2']) * np.sin(p_opt['beta_b'] / 2)
    print(f'  L_b1={p_opt["L_b1"]*1000:.1f}mm  L_b2={p_opt["L_b2"]*1000:.1f}mm  '
          f'beta_b={np.rad2deg(p_opt["beta_b"]):.1f}deg  W_bot={Wb*1000:.1f}mm')
elif p_opt['bogie_mode'] == 'frame':
    print(f'  T_b={p_opt["T_b"]*1000:.1f}mm  S_b1={p_opt["S_b1"]*1000:.1f}mm  S_b2={p_opt["S_b2"]*1000:.1f}mm')
    print(f'  th_b1={np.rad2deg(p_opt["th_b1"]):.1f}deg  th_b2={np.rad2deg(p_opt["th_b2"]):.1f}deg  j_b={p_opt["j_b"]:.2f}')

print(f'Bracket    : brk_v={p_opt["brk_v"]*1000:.1f}mm (pivot→axle V)')
print(f'P0 평지높이 : {calc_P0_height_flat(p_opt)*1000:.1f}mm')
print(f'CG 오프셋  : {p_opt["CG_offset"]*1000:.1f}mm (전방)  h_CG={p_opt["h_CG"]*1000:.1f}mm')
print(f'속도 프로파일: v_max={p_opt["v_max"]:.2f}m/s  a_lim={p_opt["a_lim"]:.2f}m/s²')
print(f'모터 한계  : 피크 {TAU_MOTOR_SAT:.2f}Nm × gear {p_opt["gear_ratio"]}:1 × η{p_opt["eta_gear"]:.2f}'
      f' = 휠 {TAU_MOTOR_SAT * p_opt["gear_ratio"] * p_opt["eta_gear"]:.1f}Nm')
print(f'모터 정격  : 연속 {p_opt["motor_tau_cont"]:.1f}Nm  Kt_eff={p_opt["Kt_eff"]:.2f}Nm/A')
print(f'배터리     : 한계 {p_opt["battery_max_current"]:.0f}A @ {p_opt["V_bus"]:.0f}V')
print(f'목적함수 값 : {f_opt:.4f}')
print('=' * 60)

# ═══════════════════════════════════════
# [SECTION 10] 저장
# ═══════════════════════════════════════
save_path = os.path.join(script_dir, 'zetin_optimal_params_v4.pkl')
with open(save_path, 'wb') as fh:
    pickle.dump({
        'p_opt': p_opt, 'x_opt': x_opt, 'f_opt': f_opt, 'elapsed': elapsed,
        'lb': lb, 'ub': ub, 'W': W, 'W_terrain': W_terrain,
        'version': 'v4_triangle_search',
    }, fh)
print(f'\n저장: {save_path}')
print(f'=== 완료 ({elapsed/60:.1f}분) ===')
