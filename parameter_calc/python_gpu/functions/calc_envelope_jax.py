"""
calc_envelope_jax: Minkowski 지형 팽창 — JAX GPU 가속 버전

원본(CPU): 이중 for 루프 → O(N * n_radius)
GPU 버전: jax.vmap + jnp 벡터 연산 → GPU 병렬 처리
"""
import jax
import jax.numpy as jnp


@jax.jit
def _envelope_single_point(i, x_t, y_t, R_w, n_radius):
    """단일 포인트의 팽창 높이 계산 (JIT 호환)"""
    # 동적 슬라이싱 대신 전체 배열에서 마스크 사용 (JIT 호환)
    dx_sq = R_w**2 - (x_t[i] - x_t)**2
    dx_sq_safe = jnp.maximum(dx_sq, 0.0)
    y_candidates = y_t + jnp.sqrt(dx_sq_safe)

    # 바퀴 반경 밖의 포인트는 -inf로 마스킹
    dist = jnp.abs(jnp.arange(x_t.shape[0]) - i)
    mask = dist <= n_radius
    y_masked = jnp.where(mask, y_candidates, -jnp.inf)

    return jnp.max(y_masked)


def calc_envelope_gpu(x_t, y_t, R_w):
    """Minkowski 팽창 — GPU 가속 버전

    Args:
        x_t: 지형 x 격자 [N] (numpy 또는 jax array)
        y_t: 지형 높이 [N]
        R_w: 바퀴 반지름

    Returns:
        y_env: 팽창된 지형 높이 [N] (numpy array)
    """
    import numpy as np

    x_t_j = jnp.array(x_t)
    y_t_j = jnp.array(y_t)
    N = len(x_t_j)
    dx = float(x_t_j[1] - x_t_j[0])
    n_radius = int(jnp.ceil(R_w / dx))

    # 모든 포인트에 대해 병렬 계산
    indices = jnp.arange(N)

    # vmap으로 N개 포인트를 동시에 계산
    _batched = jax.vmap(
        lambda i: _envelope_single_point(i, x_t_j, y_t_j, R_w, n_radius)
    )
    y_env = _batched(indices)

    return np.array(y_env)
