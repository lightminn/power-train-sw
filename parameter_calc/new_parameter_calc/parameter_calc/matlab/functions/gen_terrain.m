function [x_t, y_t] = gen_terrain(type, p)
% gen_terrain  지형 생성 함수 (통합 단일 버전)
%
% [수정 이력]
%   원본 문제 1: ZETIN_RockerBogie_Sim.m 의 로컬 버전 — L=8, N=4000
%               독립 파일(이 파일) — L=10, N=8000
%               → 두 버전이 공존해 Taguchi 비교 시 범위·해상도 불일치
%   원본 문제 2: ZETIN_RockerBogie_Sim.m 로컬 버전은 real_stairs, wood_block
%               지형 타입이 없어 'otherwise' 에러 발생
%
%   수정 후: 이 파일만 사용 (ZETIN_RockerBogie_Sim.m 의 로컬 gen_terrain 삭제 필요)
%            L=10, N=8000 단일 기준
%            모든 스크립트에서 addpath로 이 파일 포함
%
% 지원 타입:
%   'flat'        평지
%   'step'        단일 단차 (obs_h = 0.2 m)
%   'stairs'      4단 계단 (Taguchi 평가용 소형)
%   'rough'       불규칙 연속 지형 (사인파 합성)
%   'real_stairs' 실제 계단 스펙 (Rise=80mm, Depth=500mm, 3단 왕복)
%   'wood_block'  90×90mm 목재 블록 이산 노면

    % ── 지형 공통 설정 ────────────────────────────────────────
    % 모든 스크립트가 동일 L, N을 쓰도록 여기서만 정의
    % (변경 시 이 두 줄만 수정)
    L = 10;      % 지형 총 길이 [m]
    N = 8000;    % 격자 수 (계단 에지 정확도 확보)

    x_t = linspace(0, L, N);
    y_t = zeros(1, N);

    switch lower(type)

        % ── 기존 지형 ─────────────────────────────────────────
        case 'flat'
            % 기본값 0 — 아무것도 안 함

        case 'step'
            % 단일 단차 (지형 중앙, obs_h = 0.2 m)
            y_t(x_t >= L/2) = p.obs_h;

        case 'stairs'
            % 4단 소형 계단 (Taguchi 평가용, 기존 호환 유지)
            n_step = 4;
            dh = p.obs_h / n_step;   % 단당 높이 = 0.05 m
            dw = 0.25;               % 단당 수평 너비 [m]
            x0 = 2.0;
            for k = 1:n_step
                y_t(x_t >= x0 + (k-1)*dw) = k * dh;
            end

        case 'rough'
            % 불규칙 지형: 복수 사인파 합성
            % [수정] 최적화 알고리즘 교란 방지를 위해 rng() 제거
            a = 0.025;   % 기본 진폭 [m]
            y_t = a*sin(2*pi*x_t/0.9)           + ...
                  a*0.6*cos(2*pi*x_t/0.4 + 0.8)  + ...
                  a*0.4*sin(2*pi*x_t/1.5 + 2.1);
            y_t = y_t - min(y_t);   % 최저점 = 0

        % ── 신규 지형 1: 실제 계단 (경진대회 스펙) ───────────
        case 'real_stairs'
            % Rise  = 80 mm / step
            % Depth = 500 mm / tread
            % 구성: 평지(1.5m) → 3단 오르기 → 고원(1.5m) → 3단 내려오기 → 평지
            rise    = 0.080;
            depth   = 0.500;
            n_steps = 3;
            x0_up   = 1.5;

            for k = 1:n_steps
                y_t(x_t >= x0_up + (k-1)*depth) = k * rise;
            end

            x_top     = x0_up + n_steps * depth;   % = 3.0 m
            x_top_end = x_top + 1.5;               % = 4.5 m

            % [BUG-8] 내리막 계단 에지 수정:
            %   기존: 덮어쓰기 순서 오류로 내리막 에지 뭉개짐
            %   k=1: y_t(전체후방) = 2*rise, k=2: = 1*rise, k=3: = 0 으로 덮어써
            %        마지막 단이 사라지는 문제 발생
            %   수정: plateau_h 기준으로 각 단 높이 독립적으로 명시 설정
            x0_dn     = x_top_end;
            plateau_h = n_steps * rise;   % 고원 높이 = 3 × 0.08 = 0.24m

            % 고원 구간 명시 설정 (x_top ~ x_top_end)
            y_t(x_t >= x_top & x_t < x_top_end) = plateau_h;

            % 내려가는 계단: 각 에지 위치와 높이를 독립적으로 설정
            for k = 1:n_steps
                x_edge  = x0_dn + (k-1)*depth;
                h_level = plateau_h - k * rise;
                y_t(x_t >= x_edge) = max(h_level, 0);
            end

            % 마지막 계단 이후 완전 평지로 강제
            y_t(x_t >= x0_dn + n_steps*depth) = 0;
            y_t = max(y_t, 0);   % 수치 오차 방어

        % ── 신규 지형 2: 목재 블록 이산 노면 ──────────────────

        case 'wood_block'
            % 규격: 가로 90mm, 높이 40~80mm 블록
            block_w  = 0.090; 
            h_levels = [0.040, 0.050, 0.060, 0.070, 0.080];
            
            x_wood_start = 1.5; % 충분한 가속/평지 구간 확보
            x_wood_end   = L - 1.5;
            
            % 최적화 재현성을 위한 로컬 난수 스트림 (Seed 고정)
            s = RandStream('mt19937ar', 'Seed', 42);
            
            % 블록 배치를 위한 이산적 위치 계산
            % 90mm 간격으로 블록이 놓일 수 있는 자리를 미리 정의
            block_positions = x_wood_start : block_w : (x_wood_end - block_w);
            
            for j = 1:length(block_positions)
                curr_x = block_positions(j);
                
                % 블록 배치 확률 (예: 80% 확률로 블록 존재, 20%는 빈 공간)
                % 대회장이 블록으로 꽉 차 있다면 1.0으로 수정하세요.

                if rand < 1  % Seed 고정 시 수정 필요 if rand(s) < 1 <-> if rand < 1

                    % 4~8cm 중 무작위 높이 선택
                    h_cur = h_levels(randi(s, length(h_levels)));
                    
                    % 90mm 폭만큼 해당 높이 적용
                    idx = (x_t >= curr_x) & (x_t < curr_x + block_w);
                    y_t(idx) = h_cur;
                end
            end
            
            % 시작과 끝단 평지 보장
            y_t(x_t < x_wood_start) = 0;
            y_t(x_t >= x_wood_end)  = 0;

        otherwise
            error(['gen_terrain: 알 수 없는 지형 타입 "%s"\n' ...
                   '사용 가능: flat, step, stairs, rough, real_stairs, wood_block'], type);
    end
end
