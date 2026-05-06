function y_env = calc_envelope(x_t, y_t, R_w)
% calc_envelope: Minkowski Sum 기법을 이용한 지형 팽창 (Wheel Center Envelope)
% 바퀴가 모서리를 밟을 때 앞면이 닿는 현실적인 궤적을 1D 배열로 생성합니다.
    N = length(x_t);
    y_env = y_t + R_w; % 기본적으로 평지는 R_w만큼 들림
    dx = x_t(2) - x_t(1);
    n_radius = ceil(R_w / dx); 
    
    for i = 1:N
        % 탐색 윈도우 설정 (바퀴 반경 내의 지형만 탐색)
        idx_start = max(1, i - n_radius);
        idx_end   = min(N, i + n_radius);
        x_win = x_t(idx_start:idx_end);
        y_win = y_t(idx_start:idx_end);
        
        % 원의 방정식: y_center = y_terrain + sqrt(R_w^2 - dx^2)
        dy2 = R_w^2 - (x_t(i) - x_win).^2;
        dy2(dy2 < 0) = 0; % 수학적 안전장치
        y_candidates = y_win + sqrt(dy2);
        
        y_env(i) = max(y_candidates);
    end
end