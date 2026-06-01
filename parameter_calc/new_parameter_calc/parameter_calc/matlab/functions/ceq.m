function F = ceq(X, xb, F_terrain_env, p)
% ceq  역기구학 제약 방정식 (griddedInterpolant 기반 초고속 평가)

    [Wf, Wm, Wr, ~, ~] = wpos(X, xb, p);

    % [수정] F_terrain_env는 Minkowski 팽창이 적용되어 이미 R_w만큼 높아져 있음 (+ p.R_w 제거)
    hf = F_terrain_env(Wf(1));
    hm = F_terrain_env(Wm(1));
    hr = F_terrain_env(Wr(1));

    F = [Wf(2) - hf;
         Wm(2) - hm;
         Wr(2) - hr];
end