"""
newton_solver: GPU Newton-Raphson 솔버 (v5)

핵심 수정:
  - _make_sequential_solver 제거 (클로저가 매 호출마다 JIT 재컴파일 유발)
  - top-level @jax.jit 함수로 복원 → JAX가 shape 기준으로 1회만 컴파일
  - float64 제거 (GPU에서 float32만 사용)
  - CPU fsolve 2단계 복구로 실패 포인트 처리
  - [v5] 수치 자코비안 → jax.jacfwd 해석적 자코비안으로 교체
         - float32에서 엡실론 의존성 제거 → 정확도 대폭 향상
         - ceq_jax 호출 횟수 감소 (6회→3회 등가)
  - [v5] tol: 1e-7 → 1e-5 (float32에서 달성 가능한 허용오차)
  - [v5] 정규화: 1e-10 → 1e-6 (float32 정밀도에 적합)
"""
import jax
import jax.numpy as jnp
import numpy as np
from functools import partial
from scipy.optimize import fsolve
from scipy.interpolate import interp1d
from .ceq_jax import ceq_jax
from .wpos_jax import wpos_batched


# ═══════════════════════════════════════
# 해석적 자코비안 (JAX forward-mode 자동미분)
# 수치 자코비안(중심차분, eps=1e-6) 대비:
#   - float32에서 수치 소거 없음 → 정확
#   - ceq_jax 3회 등가 호출 (vs 수치 6회)
# ═══════════════════════════════════════

@jax.jit
def _analytical_jacobian(X, xb, x_t, y_t_env, p_arr):
    """JAX forward-mode 자동미분으로 3×3 자코비안 계산."""
    return jax.jacfwd(ceq_jax)(X, xb, x_t, y_t_env, p_arr)


# ═══════════════════════════════════════
# 단일 포인트 Newton 솔버
# ═══════════════════════════════════════

@jax.jit
def newton_solve_single(X0, xb, x_t, y_t_env, p_arr):
    max_iter = 20  # 해석적 자코비안으로 수렴 빨라짐
    tol = jnp.float32(1e-5)  # float32에서 달성 가능한 허용오차 (이전: 1e-7 → 불가능)

    def body_fn(carry):
        X, converged, i = carry
        F = ceq_jax(X, xb, x_t, y_t_env, p_arr)
        J = _analytical_jacobian(X, xb, x_t, y_t_env, p_arr)
        # 정규화 1e-6: float32 정밀도에서 특이 자코비안 안전하게 처리 (이전: 1e-10 → 사실상 0)
        dX = jnp.linalg.solve(J + jnp.float32(1e-6) * jnp.eye(3, dtype=jnp.float32), -F)
        step_norm = jnp.max(jnp.abs(dX))
        scale = jnp.minimum(jnp.float32(1.0), jnp.float32(1.0) / (step_norm + jnp.float32(1e-30)))
        X_new = X + scale * dX
        X_new = jnp.where(jnp.all(jnp.isfinite(X_new)), X_new, X)
        err = jnp.max(jnp.abs(F))
        return (X_new, err < tol, i + 1)

    def cond_fn(carry):
        _, converged, i = carry
        return (~converged) & (i < max_iter)

    X_final, _, _ = jax.lax.while_loop(
        cond_fn, body_fn,
        (X0, jnp.bool_(False), jnp.int32(0))
    )
    F_final = ceq_jax(X_final, xb, x_t, y_t_env, p_arr)
    converged = jnp.max(jnp.abs(F_final)) < tol
    return X_final, converged


# ═══════════════════════════════════════
# 병렬 솔버 — jax.vmap (N 포인트 동시 실행)
# lax.scan(순차) 대비 GPU에서 훨씬 빠름
# ═══════════════════════════════════════

@jax.jit
def newton_solve_parallel(X0_arr, xb_arr, x_t, y_t_env, p_arr):
    """N개 포인트를 GPU에서 동시 병렬 Newton 풀이 (jax.vmap).
    lax.scan 순차 방식 대신 모든 포인트를 한 번에 실행.
    초기값은 지형 높이 기반으로 사전 설정 필요.
    """
    def solve_one(X0, xb):
        return newton_solve_single(X0, xb, x_t, y_t_env, p_arr)
    return jax.vmap(solve_one)(X0_arr, xb_arr)


# ═══════════════════════════════════════
# 순차 warm-start 솔버 — top-level @jax.jit (재컴파일 방지)
# 병렬 솔버 실패 후 보조 수단으로 유지
# ═══════════════════════════════════════

@jax.jit
def newton_solve_sequential(X0_init, xb_arr, x_t, y_t_env, p_arr):
    """CPU kin_sim의 for 루프와 동일한 전략:
    이전 해를 다음 포인트의 초기값으로 사용 (warm-start).
    top-level @jax.jit → array shape이 같으면 재컴파일 없음.
    """
    def scan_fn(carry, xb):
        X_prev = carry
        X_sol, conv = newton_solve_single(X_prev, xb, x_t, y_t_env, p_arr)
        # 성공 시 다음 초기값 갱신, 실패 시 이전 해 유지
        X_next = jnp.where(conv, X_sol, X_prev)
        return X_next, (X_sol, conv)

    _, (X_all, conv_all) = jax.lax.scan(scan_fn, X0_init, xb_arr)
    return X_all, conv_all


# ═══════════════════════════════════════
# 2단계 복구: NaN 보간 + fsolve (CPU)
# CPU kin_sim의 _fill_nan + fsolve 재시도와 동일
# ═══════════════════════════════════════

def _fill_nan(v):
    idx = np.arange(len(v))
    vld = ~np.isnan(v)
    if np.sum(vld) < 2:
        v[~vld] = 0.0
        return v
    f = interp1d(idx[vld], v[vld], kind='nearest', fill_value='extrapolate')
    return f(idx)


def _recover_with_fsolve(X_np, conv_np, x_arr, x_t_np, y_t_np, p_arr):
    """실패 포인트를 CPU fsolve로 복구 — CPU kin_sim 2단계와 동일"""
    N = len(x_arr)
    n_fail = int(np.sum(~conv_np))
    if n_fail == 0 or n_fail == N:
        return X_np, conv_np

    # NaN 보간
    X_interp = X_np.copy()
    for col in range(3):
        vals = X_interp[:, col].copy()
        vals[~conv_np] = np.nan
        X_interp[:, col] = _fill_nan(vals)

    # fsolve 재시도 (float32로 고정)
    x_t_j = jnp.array(x_t_np, dtype=jnp.float32)
    y_t_j = jnp.array(y_t_np, dtype=jnp.float32)
    fail_idx = np.where(~conv_np)[0]
    n_recovered = 0

    for i in fail_idx:
        X0_i = X_interp[i].astype(np.float32)
        xb_i = np.float32(x_arr[i])

        def fun(X):
            res = ceq_jax(
                jnp.array(X, dtype=jnp.float32),
                jnp.array(xb_i, dtype=jnp.float32),
                x_t_j, y_t_j, p_arr
            )
            return np.array(res, dtype=np.float64)

        sol, _, ier, _ = fsolve(fun, X0_i.astype(np.float64), full_output=True)
        if ier == 1:
            X_np[i] = sol.astype(np.float32)
            conv_np[i] = True
            n_recovered += 1

    if n_recovered > 0:
        print(f'  [보간+재시도] {n_fail} 실패 → {n_recovered}개 복원')
    return X_np, conv_np


# ═══════════════════════════════════════
# 통합 kin_sim GPU 래퍼
# ═══════════════════════════════════════

def kin_sim_gpu(x_arr, x_t, y_t_env, p_arr, p_dict):
    """GPU 가속 역기구학 — 병렬 vmap 방식

    1. 지형 높이 기반 초기값 → 모든 포인트 동시 병렬 Newton (vmap)
    2. 실패 포인트 → 이웃 보간 초기값 + 재시도 (GPU 병렬)
    3. 잔여 실패 → CPU fsolve 최후 수단 (드문 경우)

    lax.scan(순차) 방식 대비:
      - GPU 병렬화로 160 포인트를 동시 처리
      - fsolve 콜백(JAX→CPU 왕복) 호출 대폭 감소
    """
    x_arr = np.atleast_1d(np.array(x_arr, dtype=np.float32))
    N = len(x_arr)

    # bb0 초기값 (CPU와 동일)
    bmode = p_dict.get('bogie_mode', p_dict.get('bogie_type', 'linear')).lower()
    if bmode == 'triangle':
        bb0 = np.float32(p_dict.get('beta_b', np.deg2rad(90))) / np.float32(2.0)
    else:
        bb0 = np.float32(0.0)

    R_w = np.float32(float(p_arr[0]))

    x_arr_j = jnp.array(x_arr, dtype=jnp.float32)
    x_t_j   = jnp.array(x_t,   dtype=jnp.float32)
    y_t_j   = jnp.array(y_t_env, dtype=jnp.float32)
    p_arr_f = p_arr.astype(jnp.float32)

    # ─── 1단계: 지형 높이 기반 초기값 → 병렬 Newton (vmap) ───
    # CPU warm-start: X0 = [R_w, 0, bb0] 고정값 사용
    # GPU 병렬: 각 포인트에서 지형 높이 기반으로 y0 초기화
    y_init = np.interp(x_arr, np.array(x_t), np.array(y_t_env)).astype(np.float32) + R_w
    X0_arr = np.column_stack([
        y_init,
        np.zeros(N, dtype=np.float32),
        np.full(N, bb0, dtype=np.float32)
    ])
    X0_arr_j = jnp.array(X0_arr, dtype=jnp.float32)

    X_sol, converged = newton_solve_parallel(X0_arr_j, x_arr_j, x_t_j, y_t_j, p_arr_f)

    X_np   = np.array(X_sol,    dtype=np.float32)
    conv_np = np.array(converged)
    n_fail = int(np.sum(~conv_np))

    # ─── 2단계: 실패 포인트 → 이웃 보간 초기값 + 재시도 (GPU 병렬) ───
    if 0 < n_fail < N:
        X_interp = X_np.copy()
        for col in range(3):
            vals = X_interp[:, col].copy()
            vals[~conv_np] = np.nan
            X_interp[:, col] = _fill_nan(vals)

        X_retry_j = jnp.array(X_interp, dtype=jnp.float32)
        X_sol2, conv2 = newton_solve_parallel(X_retry_j, x_arr_j, x_t_j, y_t_j, p_arr_f)

        improved = ~conv_np & np.array(conv2)
        X_np[improved] = np.array(X_sol2)[improved]
        conv_np = conv_np | np.array(conv2)
        n_recovered = int(np.sum(improved))
        if n_recovered > 0:
            print(f'  [재시도] {n_fail} 실패 → {n_recovered}개 복원 (GPU 병렬)')
        n_fail = int(np.sum(~conv_np))

    # ─── 3단계: 최후 수단 fsolve (매우 드문 경우) ───
    if 0 < n_fail < N:
        X_np, conv_np = _recover_with_fsolve(
            X_np, conv_np, x_arr,
            np.array(x_t_j), np.array(y_t_j), p_arr_f
        )

    # 바퀴 위치 배치 계산 (GPU)
    X_sol_f = jnp.array(X_np, dtype=jnp.float32)
    positions = wpos_batched(X_sol_f, x_arr_j, p_arr_f)
    pos_np = np.array(positions)

    R = {
        'y0':  X_np[:, 0], 'ar': X_np[:, 1], 'bb': X_np[:, 2],
        'xwf': pos_np[:, 0], 'ywf': pos_np[:, 1],
        'xwm': pos_np[:, 2], 'ywm': pos_np[:, 3],
        'xwr': pos_np[:, 4], 'ywr': pos_np[:, 5],
        'xpb': pos_np[:, 6], 'ypb': pos_np[:, 7],
        'xcg': pos_np[:, 8], 'ycg': pos_np[:, 9],
        'ok':  conv_np,
        'fail_rate': float(np.sum(~conv_np)) / N,
        'fail_idx':  np.where(~conv_np)[0],
    }

    n_final = int(np.sum(~conv_np))
    if n_final > 0:
        print(f'  [GPU kin_sim] {n_final}/{N} 포인트 수렴 실패 '
              f'(실패율 {R["fail_rate"]*100:.1f}%)')
    return R
