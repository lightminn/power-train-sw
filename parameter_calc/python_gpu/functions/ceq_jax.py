"""
ceq_jax: 역기구학 제약 방정식 — JAX GPU 가속 버전
"""
import jax
import jax.numpy as jnp
from .wpos_jax import wpos_jax


@jax.jit
def ceq_jax(X, xb, x_t, y_t_env, p_arr):
    """역기구학 제약: 3개 바퀴가 팽창 지형 위에 접촉하는 조건

    Args:
        X: [y0, ar, bb]
        xb: scalar
        x_t: 지형 x 격자 [M]
        y_t_env: 팽창 지형 높이 [M]
        p_arr: 파라미터 배열 [20]

    Returns:
        residual: [3] 제약 위반량
    """
    result = wpos_jax(X, xb, p_arr)
    Wf_x, Wf_y = result[0], result[1]
    Wm_x, Wm_y = result[2], result[3]
    Wr_x, Wr_y = result[4], result[5]

    hf = jnp.interp(Wf_x, x_t, y_t_env)
    hm = jnp.interp(Wm_x, x_t, y_t_env)
    hr = jnp.interp(Wr_x, x_t, y_t_env)

    return jnp.array([Wf_y - hf, Wm_y - hm, Wr_y - hr])
