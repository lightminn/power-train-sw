"""
validate_mujoco.py — v4 최적화 결과를 MuJoCo에서 단일 검증 (Phase 2 신뢰 버전).

목적:
  - 최적화 모델은 준정적이므로 실제 동역학과의 격차를 MuJoCo로 검증.
  - 측면뷰 절반 로봇(3륜) 모델 — 좌/우 대칭 가정.
  - 결과: 최적화 예측 vs MuJoCo 실측 토크 / 전복 여부 / 들림 비율 비교.

Phase 2 개선사항 (vs 골격):
  - 사다리꼴 속도 프로파일 (정착 → 가속 → cruise → 감속)
  - PD 속도 제어 (P+D, 게인 튜닝)
  - 0.5s settle time (정착 후 측정 시작)
  - 휠별 접촉력 직접 추출 (mj_data.contact 순회)
  - step 지형 (두 box 단차) — heightfield 미사용으로 더 견고
  - 마찰계수 명시 (지형별 μ)

향후 확장 (TODO):
  - real_stairs / wood_block / rough / curved_ramp를 heightfield로 추가
  - 좌/우 양측 + differential 메커니즘
  - 모터 토크-속도 곡선

실행:
  python validate_mujoco.py [--pkl path] [--terrain flat|step] [--duration 6.0]
"""
import argparse
import pickle
import os
import sys
import numpy as np
import mujoco


# 지형별 마찰계수 (v4_gpu.py의 MU_TERRAIN과 일치 — incline 2종 포함)
MU_TERRAIN = {
    'flat': 0.70, 'step': 0.65, 'stairs': 0.60, 'real_stairs': 0.60,
    'wood_block': 0.70, 'rough': 0.55, 'curved_ramp': 0.65,
    'incline_15': 0.65, 'incline_30': 0.60,
}


# ─── 1. MJCF 빌더 ───────────────────────────────────────

def _rocker_endpoints(p):
    """링크 끝점 상대 좌표 (P0 기준)."""
    rm = p['rocker_mode']
    brk_v = p.get('brk_v', 0.0)
    if rm == 'triangle':
        ang_pb = -p['alpha_r'] / 2.0
        ang_wr = -np.pi + p['alpha_r'] / 2.0
        Pb = np.array([p['L_r1'] * np.cos(ang_pb), p['L_r1'] * np.sin(ang_pb)])
        Wr = np.array([p['L_r2'] * np.cos(ang_wr), p['L_r2'] * np.sin(ang_wr)])
    elif rm == 'frame':
        Pb = np.array([(1 - p['j_r']) * p['T_r'] + p['S_r1'] * np.sin(p['th_r1']),
                       -p['S_r1'] * np.cos(p['th_r1'])])
        Wr = np.array([-p['j_r'] * p['T_r'] - p['S_r2'] * np.sin(p['th_r2']),
                       -p['S_r2'] * np.cos(p['th_r2'])])
    else:
        raise ValueError(f'unsupported rocker_mode: {rm}')
    Wr_axle = Wr - np.array([0.0, brk_v])
    return Pb, Wr_axle


def _bogie_endpoints(p):
    """보기 링크 끝점 상대 좌표 (Pb 기준)."""
    bm = p['bogie_mode']
    brk_v = p.get('brk_v', 0.0)
    if bm == 'triangle':
        ang_vert = -np.pi / 2.0
        ang_wf = ang_vert + p['beta_b'] / 2.0
        ang_wm = ang_vert - p['beta_b'] / 2.0
        Wf = np.array([p['L_b1'] * np.cos(ang_wf), p['L_b1'] * np.sin(ang_wf)])
        Wm = np.array([p['L_b2'] * np.cos(ang_wm), p['L_b2'] * np.sin(ang_wm)])
    elif bm == 'frame':
        Wf = np.array([(1 - p['j_b']) * p['T_b'] + p['S_b1'] * np.sin(p['th_b1']),
                       -p['S_b1'] * np.cos(p['th_b1'])])
        Wm = np.array([-p['j_b'] * p['T_b'] - p['S_b2'] * np.sin(p['th_b2']),
                       -p['S_b2'] * np.cos(p['th_b2'])])
    else:
        raise ValueError(f'unsupported bogie_mode: {bm}')
    Wf -= np.array([0.0, brk_v])
    Wm -= np.array([0.0, brk_v])
    return Wf, Wm


def build_mjcf(p, terrain='flat', terrain_obs_h=None):
    """최적화 파라미터 p로부터 MJCF XML 문자열 생성."""
    Pb_l, Wr_l = _rocker_endpoints(p)
    Wf_l, Wm_l = _bogie_endpoints(p)

    R_w = p['R_w']
    body_h = p.get('h_body', 0.3)
    body_w = 0.50
    mass = p['mass']
    m_wheel = p.get('m_wheel', 3.5)
    m_rocker_link = p.get('m_rocker_link', 1.5)
    m_bogie_link = p.get('m_bogie_link', 0.8)

    base = R_w + p.get('brk_v', 0.0)
    if p['rocker_mode'] == 'triangle':
        P0_h = base + p['L_r2'] * np.sin(p['alpha_r'] / 2.0)
    else:
        P0_h = base + p['S_r2'] * np.cos(p['th_r2'])
    chassis_z0 = P0_h + body_h / 2 + 0.05  # 본체 중심 + 5cm 안전 (지형 변동성 흡수)

    # CG_offset 적용: 본체 mass 위치를 전방으로
    CG_offset_x = p.get('CG_offset', 0.0)

    # 지형 — Phase 3+ Tier D-3-Mini: heightfield 기반으로 모든 지형 지원
    obs_h = terrain_obs_h if terrain_obs_h is not None else p.get('obs_h', 0.15)
    mu_t = MU_TERRAIN.get(terrain, 0.65)

    asset_xml = ''
    hfield_data = None  # 비-flat/step 지형에서 채워질 (nrow×ncol) 정규화 height 배열
    hfield_max_h = 1.0
    if terrain == 'flat':
        ground_xml = f'<geom name="ground" type="plane" size="20 5 0.1" rgba="0.5 0.5 0.5 1" friction="{mu_t} 0.005 0.0001"/>'
    elif terrain == 'step':
        ground_xml = f'''
        <geom name="ground_low" type="box" pos="-5 0 -0.05" size="5 5 0.05"
              rgba="0.5 0.5 0.5 1" friction="{mu_t} 0.005 0.0001"/>
        <geom name="ground_high" type="box" pos="5 0 {obs_h/2 - 0.05}" size="5 5 {obs_h/2 + 0.05}"
              rgba="0.6 0.5 0.4 1" friction="{mu_t} 0.005 0.0001"/>
        '''
    else:
        # 모든 다른 지형 → heightfield. 빌드 시 hfield 메타 정의, 데이터는 시뮬 실행 전에 채움.
        from functions.gen_terrain import gen_terrain as _gt
        x_terrain, y_terrain = _gt(terrain, p)
        hfield_max_h = max(float(np.max(y_terrain)), 0.05)
        x_min, x_max = float(x_terrain[0]), float(x_terrain[-1])
        x_radius = (x_max - x_min) / 2.0
        x_center = (x_max + x_min) / 2.0
        # hfield 해상도: ncol(x), nrow(y). y는 단순 extrude.
        ncol = 512
        nrow = 8
        # x 그리드 재샘플링 후 [0, 1]로 정규화
        x_resample = np.linspace(x_min, x_max, ncol)
        y_resample = np.interp(x_resample, x_terrain, y_terrain)
        height_normalized = y_resample / hfield_max_h
        # 2D 배열 (nrow×ncol), y방향 동일 (extrude)
        hfield_data = np.tile(height_normalized[None, :], (nrow, 1)).astype(np.float32)

        asset_xml = (
            f'<asset><hfield name="terrain_hf" nrow="{nrow}" ncol="{ncol}" '
            f'size="{x_radius} 2.5 {hfield_max_h} 0.05"/></asset>'
        )
        ground_xml = (
            f'<geom name="terrain_geom" type="hfield" hfield="terrain_hf" '
            f'pos="{x_center} 0 0" rgba="0.5 0.45 0.4 1" '
            f'friction="{mu_t} 0.005 0.0001"/>'
        )

    # 본체 mass = 전체 - 휠*6 - 로커*2 - 보기*2 (측면뷰는 1/2지만 mass 분배는 보수적으로 전체로 둠)
    body_mass = max(mass - 6 * m_wheel - 2 * m_rocker_link - 2 * m_bogie_link, 5.0)

    # 본체 inertia (간단 box 근사): 실제 mujoco가 box geom mass로 자동 계산
    # CG_offset_x는 inertia 박스를 약간 전방으로 이동
    xml = f'''<mujoco model="rocker_bogie_side">
  <option timestep="0.0005" gravity="0 0 -9.81" integrator="RK4"/>
  <default>
    <joint damping="0.1" armature="0.001"/>
    <geom contype="1" conaffinity="1"/>
  </default>

  {asset_xml}

  <worldbody>
    {ground_xml}

    <body name="chassis" pos="0 0 {chassis_z0}">
      <joint name="slide_x" type="slide" axis="1 0 0"/>
      <joint name="slide_z" type="slide" axis="0 0 1"/>
      <joint name="pitch"   type="hinge" axis="0 1 0"/>
      <geom name="body_geom" type="box" size="{body_w/2} 0.10 {body_h/2}"
            mass="{body_mass:.2f}" pos="{CG_offset_x} 0 0"
            rgba="0.2 0.5 0.8 0.5"/>

      <body name="rocker" pos="0 0 {-body_h/2}">
        <joint name="rocker_pitch" type="hinge" axis="0 1 0" range="-1.5 1.5"/>
        <geom type="capsule" fromto="0 0 0  {Pb_l[0]} 0 {Pb_l[1]}" size="0.012"
              mass="{m_rocker_link/2:.2f}" rgba="0.7 0.3 0.3 1"/>
        <geom type="capsule" fromto="0 0 0  {Wr_l[0]} 0 {Wr_l[1]}" size="0.012"
              mass="{m_rocker_link/2:.2f}" rgba="0.7 0.3 0.3 1"/>

        <body name="wheel_rear" pos="{Wr_l[0]} 0 {Wr_l[1]}">
          <joint name="wheel_r_spin" type="hinge" axis="0 1 0" damping="0.02"/>
          <geom name="wheel_r_geom" type="cylinder" size="{R_w} 0.04" euler="1.5708 0 0"
                mass="{m_wheel}" rgba="0.2 0.2 0.2 1"
                friction="{mu_t} 0.005 0.0001"/>
        </body>

        <body name="bogie" pos="{Pb_l[0]} 0 {Pb_l[1]}">
          <joint name="bogie_pitch" type="hinge" axis="0 1 0" range="-1.5 1.5"/>
          <geom type="capsule" fromto="0 0 0  {Wf_l[0]} 0 {Wf_l[1]}" size="0.010"
                mass="{m_bogie_link/2:.2f}" rgba="0.3 0.6 0.3 1"/>
          <geom type="capsule" fromto="0 0 0  {Wm_l[0]} 0 {Wm_l[1]}" size="0.010"
                mass="{m_bogie_link/2:.2f}" rgba="0.3 0.6 0.3 1"/>

          <body name="wheel_front" pos="{Wf_l[0]} 0 {Wf_l[1]}">
            <joint name="wheel_f_spin" type="hinge" axis="0 1 0" damping="0.02"/>
            <geom name="wheel_f_geom" type="cylinder" size="{R_w} 0.04" euler="1.5708 0 0"
                  mass="{m_wheel}" rgba="0.2 0.2 0.2 1"
                  friction="{mu_t} 0.005 0.0001"/>
          </body>

          <body name="wheel_mid" pos="{Wm_l[0]} 0 {Wm_l[1]}">
            <joint name="wheel_m_spin" type="hinge" axis="0 1 0" damping="0.02"/>
            <geom name="wheel_m_geom" type="cylinder" size="{R_w} 0.04" euler="1.5708 0 0"
                  mass="{m_wheel}" rgba="0.2 0.2 0.2 1"
                  friction="{mu_t} 0.005 0.0001"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="m_f" joint="wheel_f_spin" gear="{p.get('gear_ratio', 1)}" ctrllimited="true" ctrlrange="-{p.get('motor_tau_peak', 39.0)} {p.get('motor_tau_peak', 39.0)}"/>
    <motor name="m_m" joint="wheel_m_spin" gear="{p.get('gear_ratio', 1)}" ctrllimited="true" ctrlrange="-{p.get('motor_tau_peak', 39.0)} {p.get('motor_tau_peak', 39.0)}"/>
    <motor name="m_r" joint="wheel_r_spin" gear="{p.get('gear_ratio', 1)}" ctrllimited="true" ctrlrange="-{p.get('motor_tau_peak', 39.0)} {p.get('motor_tau_peak', 39.0)}"/>
  </actuator>
</mujoco>
'''
    return xml, hfield_data


# ─── 2. 사다리꼴 속도 ramp ──────────────────────────────

def trap_ramp(t, t_settle, t_accel, t_cruise, t_decel, v_max):
    """0→v_max→0 사다리꼴 ramp. t_settle 이전엔 0."""
    if t < t_settle:
        return 0.0
    t_phase = t - t_settle
    if t_phase < t_accel:
        return v_max * t_phase / t_accel
    elif t_phase < t_accel + t_cruise:
        return v_max
    elif t_phase < t_accel + t_cruise + t_decel:
        return v_max * (1.0 - (t_phase - t_accel - t_cruise) / t_decel)
    else:
        return 0.0


# ─── 3. 시뮬레이션 + 휠별 접촉력 ─────────────────────────

def get_wheel_contact_force(model, data, geom_name):
    """특정 휠 geom의 접지 수직 항력 (N) 합산."""
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    total_n = 0.0
    for ci in range(data.ncon):
        c = data.contact[ci]
        if c.geom1 == gid or c.geom2 == gid:
            # contact frame: c.frame[0:3]이 법선. force[0]이 normal force.
            fc = np.zeros(6, dtype=np.float64)
            mujoco.mj_contactForce(model, data, ci, fc)
            total_n += abs(fc[0])  # 첫 성분이 normal
    return total_n


def simulate_run(xml, hfield_data=None, duration=6.0, v_target=0.8, t_settle=0.5,
                 kp=1.2, kd=0.0, log_dt=0.01, verbose=True,
                 R_w=0.10, tau_clip=39.0):
    """ramp 제어 + 부드러운 P 추종으로 시뮬. 휠별 접촉력 로깅 포함.

    Args:
        xml: MJCF 문자열
        hfield_data: (nrow×ncol) [0,1] 정규화 높이 배열. None이면 무시.
        나머지: 시뮬 설정.

    게인 튜닝 노트:
      - Kp=1.2, Kd=0: 토크 포화 회피, 추종 속도 ~90% 안쪽 유지.
    """
    model = mujoco.MjModel.from_xml_string(xml)
    # hfield 데이터 채움
    if hfield_data is not None and model.nhfield > 0:
        flat = hfield_data.flatten().astype(np.float32)
        # mujoco의 hfield_data 인덱스 — 보통 0번 hfield
        adr = int(model.hfield_adr[0])
        size = int(model.hfield_nrow[0] * model.hfield_ncol[0])
        model.hfield_data[adr:adr + size] = flat[:size]
    data = mujoco.MjData(model)

    dt = model.opt.timestep
    n_steps = int(duration / dt)
    log_every = max(1, int(log_dt / dt))
    n_log = n_steps // log_every + 1

    log = {
        't': np.zeros(n_log),
        'v_target': np.zeros(n_log),
        'x_chassis': np.zeros(n_log),
        'z_chassis': np.zeros(n_log),
        'pitch': np.zeros(n_log),
        'tau': np.zeros((n_log, 3)),
        'wheel_v_actual': np.zeros((n_log, 3)),
        'wheel_omega': np.zeros((n_log, 3)),
        'contact_N': np.zeros((n_log, 3)),  # front/mid/rear normal force
    }

    aid_f = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'm_f')
    aid_m = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'm_m')
    aid_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'm_r')
    jid_x = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'slide_x')
    jid_z = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'slide_z')
    jid_p = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'pitch')
    jid_wf = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'wheel_f_spin')
    jid_wm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'wheel_m_spin')
    jid_wr = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'wheel_r_spin')
    qx = model.jnt_qposadr[jid_x]
    qz = model.jnt_qposadr[jid_z]
    qp = model.jnt_qposadr[jid_p]
    qvf = model.jnt_dofadr[jid_wf]
    qvm = model.jnt_dofadr[jid_wm]
    qvr = model.jnt_dofadr[jid_wr]

    # R_w·tau_clip은 호출부(main)에서 p_opt 기준으로 전달됨 (기본값은 BL70200 p0).

    # ramp 일정: settle → accel → cruise → decel
    t_accel = 1.0
    t_cruise = max(duration - t_settle - 2 * t_accel, 1.0)
    t_decel = 1.0

    prev_err = np.zeros(3)
    li = 0
    for k in range(n_steps):
        t_now = data.time

        # 목표 휠 속도 (선속도/R_w = 각속도)
        v_t = trap_ramp(t_now, t_settle, t_accel, t_cruise, t_decel, v_target)
        omega_target = v_t / R_w

        # PD: τ = Kp·e + Kd·de/dt
        ws = np.array([data.qvel[qvf], data.qvel[qvm], data.qvel[qvr]])
        err = omega_target - ws
        derr = (err - prev_err) / dt
        prev_err = err
        ctrl = np.clip(kp * err + kd * derr, -tau_clip, tau_clip)
        data.ctrl[aid_f] = ctrl[0]
        data.ctrl[aid_m] = ctrl[1]
        data.ctrl[aid_r] = ctrl[2]

        mujoco.mj_step(model, data)

        if k % log_every == 0 and li < n_log:
            log['t'][li] = data.time
            log['v_target'][li] = v_t
            log['x_chassis'][li] = data.qpos[qx]
            log['z_chassis'][li] = data.qpos[qz]
            log['pitch'][li] = data.qpos[qp]
            log['tau'][li] = ctrl
            log['wheel_omega'][li] = ws
            log['wheel_v_actual'][li] = ws * R_w
            log['contact_N'][li, 0] = get_wheel_contact_force(model, data, 'wheel_f_geom')
            log['contact_N'][li, 1] = get_wheel_contact_force(model, data, 'wheel_m_geom')
            log['contact_N'][li, 2] = get_wheel_contact_force(model, data, 'wheel_r_geom')
            li += 1

    # log trim
    for k in log:
        log[k] = log[k][:li]

    if verbose:
        print(f'  스텝: {n_steps}회 × {dt*1000:.1f}ms = {duration:.2f}s')
        print(f'  로깅: {li}회 × {log_dt*1000:.1f}ms')

    return log


# ─── 4. 메인 ─────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument('--pkl', default=os.path.join(script_dir, 'zetin_optimal_params_v4.pkl'))
    ap.add_argument('--terrain', default='flat',
                    choices=['flat', 'step', 'stairs', 'real_stairs',
                             'rough', 'wood_block', 'curved_ramp',
                             'incline_15', 'incline_30'])
    ap.add_argument('--duration', type=float, default=6.0)
    ap.add_argument('--v_target', type=float, default=0.8)
    ap.add_argument('--save_xml', default=None)
    ap.add_argument('--save_log', default=None, help='log npz 저장 경로')
    args = ap.parse_args()

    print(f'pkl 로드: {args.pkl}')
    if not os.path.isfile(args.pkl):
        sys.exit(f'[오류] pkl 파일 없음: {args.pkl}\n'
                 '       먼저 ZETIN_JointOptSearch_v4_gpu.py 를 실행해 결과를 생성하세요.')
    with open(args.pkl, 'rb') as fh:
        result = pickle.load(fh)
    p_opt = result['p_opt']

    print(f'최적 구조: rocker={p_opt["rocker_mode"]} bogie={p_opt["bogie_mode"]}')
    print(f'           brk_v={p_opt.get("brk_v",0)*1000:.1f}mm  CG_offset={p_opt.get("CG_offset",0)*1000:.1f}mm')
    print(f'           f_opt={result["f_opt"]:.4f}')

    print(f'\nMJCF 빌드 (terrain={args.terrain}, μ={MU_TERRAIN.get(args.terrain, 0.65)})...')
    xml, hfield_data = build_mjcf(p_opt, terrain=args.terrain)
    if hfield_data is not None:
        print(f'  hfield: {hfield_data.shape}  max_h={hfield_data.max():.3f} (정규화)')
    if args.save_xml:
        with open(args.save_xml, 'w') as fh:
            fh.write(xml)
        print(f'  MJCF 저장: {args.save_xml}')

    print(f'\nMuJoCo 시뮬 (duration={args.duration}s, v={args.v_target}m/s, settle=0.5s, ramp 1+cruise+1)...')
    log = simulate_run(xml, hfield_data=hfield_data, duration=args.duration, v_target=args.v_target,
                       R_w=p_opt.get('R_w', 0.10), tau_clip=p_opt.get('motor_tau_peak', 39.0))

    # ── 결과 분석 (settle 이후 데이터만) ──
    t = log['t']
    settle_mask = t >= 0.5
    if not np.any(settle_mask):
        print('경고: settle 기간 데이터만 있음.')
        return

    tau_settle = log['tau'][settle_mask]
    v_actual = log['wheel_v_actual'][settle_mask]
    v_tgt = log['v_target'][settle_mask]
    pitch_settle = log['pitch'][settle_mask]
    cN = log['contact_N'][settle_mask]
    x = log['x_chassis']

    motor_peak = p_opt.get('motor_tau_peak', 39.0)
    tau_peak_each = np.max(np.abs(tau_settle), axis=0)
    tau_peak_overall = float(np.max(tau_peak_each))
    sat_pct = tau_peak_overall / motor_peak * 100
    # 추적 오차: 측정 시점에서 평균 휠 속도와 목표 차이
    v_mean_actual = float(np.mean(np.mean(v_actual, axis=1)[v_tgt > 0.05]))
    v_mean_tgt = float(np.mean(v_tgt[v_tgt > 0.05]))
    pitch_range = (float(np.degrees(np.min(pitch_settle))),
                   float(np.degrees(np.max(pitch_settle))))
    pitch_amp = pitch_range[1] - pitch_range[0]

    # 휠별 접촉력 통계
    cN_mean = np.mean(cN, axis=0)
    cN_min = np.min(cN, axis=0)

    # liftoff: contact_N < threshold(예: 5N)이면 들림
    LIFTOFF_THR = 5.0
    liftoff_pct = np.mean(cN < LIFTOFF_THR, axis=0) * 100

    print('\n' + '=' * 64)
    print('  MuJoCo 실측 (settle 이후 통계)')
    print('-' * 64)
    print(f'주행거리 (총): {(x[-1]-x[0])*1000:.1f}mm  최종 x={x[-1]:.3f}m')
    print(f'평균 속도   : 목표 {v_mean_tgt:.3f}m/s  실측 {v_mean_actual:.3f}m/s  '
          f'({v_mean_actual/v_mean_tgt*100:.0f}% 추종)')
    print(f'피크 모터 τ : F={tau_peak_each[0]:.2f}  M={tau_peak_each[1]:.2f}  R={tau_peak_each[2]:.2f}  Nm')
    print(f'             최대 {tau_peak_overall:.2f}Nm  (한계 {motor_peak:.1f}Nm 대비 {sat_pct:.0f}%)')
    print(f'피치 진폭   : {pitch_range[0]:+.1f}° ~ {pitch_range[1]:+.1f}° (Δ={pitch_amp:.1f}°)')
    print(f'접촉력 평균 : F={cN_mean[0]:.1f}  M={cN_mean[1]:.1f}  R={cN_mean[2]:.1f}  N')
    print(f'접촉력 최소 : F={cN_min[0]:.1f}  M={cN_min[1]:.1f}  R={cN_min[2]:.1f}  N')
    print(f'들림 비율   : F={liftoff_pct[0]:4.1f}%  M={liftoff_pct[1]:4.1f}%  R={liftoff_pct[2]:4.1f}%  '
          f'(<{LIFTOFF_THR}N 기준)')
    print('=' * 64)

    # 검증 신호등
    print('\n  검증 신호등:')
    print(f'    속도 추종: {"✓ 양호" if abs(v_mean_actual - v_mean_tgt) < 0.1 else "△ 마진" if abs(v_mean_actual - v_mean_tgt) < 0.25 else "✗ 불량"}')
    print(f'    모터 토크: {"✓ 충분" if sat_pct < 80 else "△ 마진" if sat_pct < 100 else "✗ 포화"}')
    print(f'    안정 자세: {"✓ 안정" if pitch_amp < 5 else "△ 진동" if pitch_amp < 15 else "✗ 불안정"}')
    print(f'    접지 유지: {"✓ 충실" if max(liftoff_pct) < 5 else "△ 일시들림" if max(liftoff_pct) < 20 else "✗ 빈번이격"}')
    print('=' * 64)

    if args.save_log:
        np.savez(args.save_log, **log)
        print(f'\n로그 저장: {args.save_log}')


if __name__ == '__main__':
    main()
