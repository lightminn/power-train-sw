"""
ceq: 역기구학 제약 방정식 (interpolant 기반)
"""
import numpy as np
from .wpos import wpos


def ceq(X, xb, F_terrain_env, p):
    Wf, Wm, Wr, _, _ = wpos(X, xb, p)

    hf = F_terrain_env(Wf[0])
    hm = F_terrain_env(Wm[0])
    hr = F_terrain_env(Wr[0])

    return np.array([Wf[1] - hf, Wm[1] - hm, Wr[1] - hr])
