% ============================================================
% ZETIN_Animation_v3.m
% 최적 Rocker-Bogie (Frame 구조) — 4종 지형 주행 애니메이션
%
% [업데이트 내역]
% 1. Minkowski 지형 팽창(Envelope)을 적용하여 바퀴가 파묻히는 현상 해결
% 2. 렌더링 속도 대폭 향상 (강제 pause 제거 및 프레임 최적화)
% ============================================================
clear; clc; close all;
script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);
addpath(fullfile(script_dir, 'functions'));

%% ─────────────────────────────────────────
%% 최적 파라미터 로드
%% ─────────────────────────────────────────
mat_path = fullfile(script_dir, 'zetin_optimal_params_v3.mat');
if ~exist(mat_path,'file')
    error('zetin_optimal_params_v3.mat 파일이 없습니다. 최적화를 먼저 실행하세요.');
end
load(mat_path, 'p_opt');
fprintf('\n======================================================\n');
fprintf(' 파라미터 로드 완료 (R_w=%.0fmm, mass=%.1fkg)\n', p_opt.R_w*1000, p_opt.mass);
fprintf('  Rocker 모드: %s\n', p_opt.rocker_mode);
fprintf('  Bogie 모드 : %s\n', p_opt.bogie_mode);
fprintf('======================================================\n');

%% ─────────────────────────────────────────
%% 애니메이션 설정
%% ─────────────────────────────────────────
SAVE_VIDEO  = true;    % MP4 저장 여부
FPS         = 60;      
N_FRAMES    = 180;     % [수정] 속도를 위해 프레임 수 180으로 최적화 (기존 300)
PAUSE_BTWN  = 1.0;     

terrain_list  = {'real_stairs', 'wood_block', 'rough', 'step'};
terrain_names = {'실제 계단 (Rise 80mm × 3단)', '목재 블록 (40~80mm 불규칙)', ...
                 '불규칙 지형 (사인파 합성)',     '단차 150mm'};
terrain_cols  = {[0.20 0.55 0.35], [0.80 0.50 0.15], ...
                 [0.25 0.40 0.80], [0.70 0.25 0.20]};

%% ─────────────────────────────────────────
%% 색상 팔레트
%% ─────────────────────────────────────────
C_GROUND  = [0.78 0.68 0.58];   
C_ROCKER  = [0.20 0.38 0.72];   
C_BOGIE   = [0.18 0.62 0.38];   
C_WHEEL   = [0.18 0.18 0.18];   
C_NR      = [0.10 0.45 0.75];   
C_NM      = [0.10 0.60 0.25];   
C_NF      = [0.85 0.40 0.10];   

%% ─────────────────────────────────────────
%% 메인 애니메이션 루프
%% ─────────────────────────────────────────
fig = figure('Name','ZETIN 최적 구조 주행 애니메이션', ...
             'Position',[40 40 1400 800], 'Color',[0.10 0.10 0.12]);
             
for ti = 1:length(terrain_list)
    terrain = terrain_list{ti};
    t_name  = terrain_names{ti};
    t_col   = terrain_cols{ti};
    fprintf('\n[%d/4] %s 시뮬레이션 중...', ti, terrain);
    
    %% 기구학 및 동역학 시뮬레이션 (팽창 지형 적용!)
    [x_t, y_t_raw] = gen_terrain(terrain, p_opt);
    
    % [핵심 수정] 바퀴 중심이 이동할 '팽창 지형' 생성
    y_t_env = calc_envelope(x_t, y_t_raw, p_opt.R_w);
    
    cb    = get_cb_fwd_local(p_opt);
    a_eff = get_a_eff_local(p_opt);
    b_eff = get_b_eff_local(p_opt);
    
    xs = x_t(1)   + b_eff + 0.05;
    xe = x_t(end) - (a_eff + cb) - 0.05;
    xa = linspace(xs, xe, N_FRAMES);
    
    % [수정] 기구학과 동역학은 팽창 지형(y_t_env)에서 품!
    R = kin_sim(xa, x_t, y_t_env, p_opt);
    D = calc_dynamics(R, xa, x_t, y_t_env, p_opt); 
    
    fprintf(' 완료 (기구학 실패율=%.1f%%)\n', R.fail_rate*100);
    
    %% 비디오 라이터 초기화
    if SAVE_VIDEO
        vid_name = fullfile(script_dir, sprintf('ZETIN_animation_%s.mp4', terrain));
        vw = VideoWriter(vid_name, 'MPEG-4');
        vw.FrameRate = FPS;
        open(vw);
    end
    
    %% y 축 범위 계산 (화면에 그리는 건 원본 지형 기준)
    y_min = -0.08;
    y_max = max(y_t_raw) + p_opt.R_w*4 + p_opt.h_body + 0.20;
    
    %% 프레임 루프
    for fi = 1:N_FRAMES
        clf(fig);
        
        %% ── 레이아웃: 상단(로봇 자세) + 하단(법선력 분포) ──
        ax1 = subplot('Position',[0.03 0.38 0.94 0.57]);
        hold(ax1,'on'); box(ax1,'on');
        set(ax1,'Color',[0.08 0.08 0.10],'XColor',[0.8 0.8 0.8],...
                'YColor',[0.8 0.8 0.8],'FontSize',8);
                
        % [수정] 화면에 그리는 진짜 땅은 원본 지형(y_t_raw) 사용
        x_vis_start = max(x_t(1),   xa(fi) - 1.5);
        x_vis_end   = min(x_t(end), xa(fi) + 2.0);
        mask_vis = x_t >= x_vis_start & x_t <= x_vis_end;
        x_v = x_t(mask_vis); y_v = y_t_raw(mask_vis); 
        
        if ~isempty(x_v)
            fill(ax1, [x_v,fliplr(x_v)], [y_v, y_min*ones(1,length(y_v))], ...
                 C_GROUND,'EdgeColor','none','FaceAlpha',0.75);
            plot(ax1, x_v, y_v, '-','Color',[0.5 0.4 0.3],'LineWidth',1.5);
        end
        
        if fi > 1
            trail_len = min(fi-1, 80);
            idx_trail = max(1,fi-trail_len):fi-1;
            valid = R.ok(idx_trail);
            plot(ax1, xa(idx_trail(valid)), R.y0(idx_trail(valid)), ...
                 '-','Color',[t_col, 0.6],'LineWidth',1.5);
        end
        
        if R.ok(fi)
            draw_robot_v4(ax1, xa(fi), R.y0(fi), R.ar(fi), R.bb(fi), p_opt, C_ROCKER, C_BOGIE);
        end
        
        plot(ax1, [xa(fi),xa(fi)],[y_min, R.y0(fi)], ...
             ':','Color',[1 1 1 0.25],'LineWidth',1);
        xlim(ax1, [x_vis_start, x_vis_end]);
        ylim(ax1, [y_min, y_max]);
        ylabel(ax1, 'Y [m]','FontSize',9,'Color',[0.8 0.8 0.8]);
        
        prog = (fi-1)/(N_FRAMES-1)*100;
        title(ax1, sprintf('%s   [%d/%d]  진행률: %.0f%%', t_name, ti, 4, prog), ...
              'FontSize',11,'FontWeight','bold','Color','w');
              
        info_str = build_info_str(p_opt);
        text(ax1, x_vis_end-0.05, y_max-0.02, info_str, ...
             'FontSize',8,'Color',[0.9 0.9 0.9],'HorizontalAlignment','right',...
             'VerticalAlignment','top','HandleVisibility','off',...
             'BackgroundColor',[0 0 0 0.5],'Margin',4);
             
        %% ── 하단 패널 1: 법선력 히스토리 (시계열) ──
        ax2 = subplot('Position',[0.03 0.06 0.55 0.27]);
        hold(ax2,'on'); box(ax2,'on');
        set(ax2,'Color',[0.08 0.08 0.10],'XColor',[0.7 0.7 0.7],...
                'YColor',[0.7 0.7 0.7],'FontSize',8);
                
        if fi > 1
            idx_h = 1:fi;
            valid = R.ok(idx_h);
            plot(ax2, xa(idx_h(valid)), D.Nr(idx_h(valid))*1000,'--','Color',[C_NR, 0.7],'LineWidth',1.2);
            plot(ax2, xa(idx_h(valid)), D.Nm(idx_h(valid))*1000,'--','Color',[C_NM, 0.7],'LineWidth',1.2);
            plot(ax2, xa(idx_h(valid)), D.Nf(idx_h(valid))*1000,'--','Color',[C_NF, 0.7],'LineWidth',1.2);
        end
        
        yline(ax2, p_opt.mass*p_opt.g/3*1000,'--','Color',[0.5 0.5 0.5],'LineWidth',1.0,'HandleVisibility','off');
        
        if R.ok(fi)
            plot(ax2, xa(fi), D.Nr(fi)*1000,'o','MarkerSize',8,'MarkerFaceColor',C_NR,'MarkerEdgeColor','w','LineWidth',1.0,'DisplayName','Nr 뒷바퀴');
            plot(ax2, xa(fi), D.Nm(fi)*1000,'o','MarkerSize',8,'MarkerFaceColor',C_NM,'MarkerEdgeColor','w','LineWidth',1.0,'DisplayName','Nm 중간바퀴');
            plot(ax2, xa(fi), D.Nf(fi)*1000,'o','MarkerSize',8,'MarkerFaceColor',C_NF,'MarkerEdgeColor','w','LineWidth',1.0,'DisplayName','Nf 앞바퀴');
        end
        
        xlim(ax2,[xs, xe]);
        ylabel(ax2,'법선력 [mN]','FontSize',9,'Color',[0.7 0.7 0.7]);
        title(ax2,'각 바퀴의 법선력 분배 곡선','FontSize',10,'Color',[0.8 0.8 0.8]);
        if fi==1, legend(ax2,'Location','northwest','FontSize',8,'TextColor','w'); end
        
        %% ── 하단 패널 2: 현재 법선력 실시간 바(Bar) 그래프 ──
        ax3 = subplot('Position',[0.63 0.06 0.34 0.27]);
        hold(ax3,'on'); box(ax3,'on');
        set(ax3,'Color',[0.08 0.08 0.10],'XColor',[0.7 0.7 0.7],...
                'YColor',[0.7 0.7 0.7],'FontSize',8);
                
        N_eq = p_opt.mass*p_opt.g/3 * 1000;
        if R.ok(fi)
            N_vals = [D.Nr(fi), D.Nm(fi), D.Nf(fi)] * 1000;
            bar_cols = [C_NR; C_NM; C_NF];
            for bi2=1:3
                bar(ax3, bi2, N_vals(bi2), 0.6, 'FaceColor', bar_cols(bi2,:), 'EdgeColor','none');
            end
            
            if mean(N_vals) > 0.1
                imbal_cur = (max(N_vals)-min(N_vals))/mean(N_vals)*100;
            else
                imbal_cur = 0;
            end
            title(ax3, sprintf('현재 편중도: %.1f%%', imbal_cur), 'FontSize',10,'Color',[0.8 0.8 0.8],'FontWeight','bold');
        else
            title(ax3, '기구학 수렴 실패', 'FontSize',10,'Color','r','FontWeight','bold');
        end
        
        yline(ax3, N_eq,'--','Color',[0.6 0.6 0.6],'LineWidth',1.5);
        ylim(ax3, [0, N_eq * 2.5]);
        set(ax3,'XTick',1:3,'XTickLabel',{'Nr(뒤)','Nm(중)','Nf(앞)'},'FontSize',9);
        ylabel(ax3,'법선력 [mN]','FontSize',9,'Color',[0.7 0.7 0.7]);
        
        %% 프레임 렌더링 및 저장
        % [수정] limitrate를 추가하여 화면 출력 병목을 대폭 줄임
        drawnow limitrate; 
        
        if SAVE_VIDEO
            writeVideo(vw, getframe(fig));
        end
        
        % [수정] 강제로 화면을 멈추게 하던 pause 명령줄 삭제 (속도 향상)
    end
    
    if SAVE_VIDEO
        close(vw);
        fprintf('  저장 완료: ZETIN_animation_%s.mp4\n', terrain);
    end
    if ti < length(terrain_list)
        pause(PAUSE_BTWN);
    end
end
fprintf('\n=== 모든 애니메이션 완료 ===\n');

%% ═════════════════════════════════════════════════════════════
%% 헬퍼: 바퀴 그리기
%% ═════════════════════════════════════════════════════════════
function draw_wheel(ax, cx, cy, r, col)
    th = linspace(0, 2*pi, 32);
    fill(ax, cx+r*cos(th), cy+r*sin(th), col, ...
         'EdgeColor','none','FaceAlpha',0.85);
    % 스포크 (회전감)
    for ang = 0:pi/3:pi-0.01
        plot(ax, [cx+r*0.2*cos(ang), cx+r*0.85*cos(ang)], ...
                 [cy+r*0.2*sin(ang), cy+r*0.85*sin(ang)], ...
             '-','Color',[1 1 1]*0.55,'LineWidth',0.7);
        plot(ax, [cx+r*0.2*cos(ang+pi), cx+r*0.85*cos(ang+pi)], ...
                 [cy+r*0.2*sin(ang+pi), cy+r*0.85*sin(ang+pi)], ...
             '-','Color',[1 1 1]*0.55,'LineWidth',0.7);
    end
end

%% ═════════════════════════════════════════════════════════════
%% 헬퍼: Frame-Frame 완벽 렌더링 (wpos 연동)
%% ═════════════════════════════════════════════════════════════
function draw_robot_v4(ax, xb, y0, ar, bb, p, col_r, col_b)
    X = [y0; ar; bb];
    [Wf, Wm, Wr, Pb, ~] = wpos(X, xb, p);
    P0 = [xb; y0];
    hv = {'HandleVisibility','off'};
    
    switch lower(p.rocker_mode)
        case {'linear','triangle'}
            plot(ax, [Wr(1),P0(1),Pb(1)],[Wr(2),P0(2),Pb(2)],'-','Color',col_r,'LineWidth',4.0, hv{:});
        case 'frame'
            ar_e = ar + p.phi_r0;
            ur = [cos(ar_e); sin(ar_e)];
            Ptr = P0 - p.j_r * p.T_r * ur;
            Ptf = P0 + (1-p.j_r) * p.T_r * ur;
            plot(ax, [Ptr(1),Ptf(1)],[Ptr(2),Ptf(2)],'-','Color',col_r,'LineWidth',4.0,hv{:});
            plot(ax, [Ptf(1),Pb(1)],[Ptf(2),Pb(2)],'-','Color',col_r,'LineWidth',3.0,hv{:});
            plot(ax, [Ptr(1),Wr(1)],[Ptr(2),Wr(2)],'-','Color',col_r,'LineWidth',3.0,hv{:});
    end
    
    switch lower(p.bogie_mode)
        case {'linear','triangle'}
            plot(ax, [Wm(1),Pb(1),Wf(1)],[Wm(2),Pb(2),Wf(2)],'-','Color',col_b,'LineWidth',4.0,hv{:});
        case 'frame'
            ubb = [cos(bb); sin(bb)];
            Pbm = Pb - p.j_b * p.T_b * ubb;
            Pbf = Pb + (1-p.j_b) * p.T_b * ubb;
            plot(ax, [Pbm(1),Pbf(1)],[Pbm(2),Pbf(2)],'-','Color',col_b,'LineWidth',4.0,hv{:});
            plot(ax, [Pbf(1),Wf(1)],[Pbf(2),Wf(2)],'-','Color',col_b,'LineWidth',3.0,hv{:});
            plot(ax, [Pbm(1),Wm(1)],[Pbm(2),Wm(2)],'-','Color',col_b,'LineWidth',3.0,hv{:});
    end
    
    bw = 0.08; bh = p.h_body;
    ar_e = ar + p.phi_r0;
    ur = [cos(ar_e); sin(ar_e)]; 
    nr = [-sin(ar_e); cos(ar_e)];
    corners_x = [P0(1)-bw*ur(1), P0(1)+bw*ur(1), P0(1)+bw*ur(1)+bh*nr(1), P0(1)-bw*ur(1)+bh*nr(1)];
    corners_y = [P0(2)-bw*ur(2), P0(2)+bw*ur(2), P0(2)+bw*ur(2)+bh*nr(2), P0(2)-bw*ur(2)+bh*nr(2)];
    patch(ax, corners_x, corners_y, [0.4 0.6 0.85], 'FaceAlpha',0.55,'EdgeColor',[0.1 0.3 0.7],'LineWidth',2.0, hv{:});
    
    plot(ax, P0(1),P0(2),'o','MarkerSize',8,'MarkerFaceColor',col_r,'MarkerEdgeColor','w','LineWidth',1.5,hv{:});
    plot(ax, Pb(1),Pb(2),'o','MarkerSize',6,'MarkerFaceColor',col_b,'MarkerEdgeColor','w','LineWidth',1.5,hv{:});
    
    draw_wheel(ax, Wf(1),Wf(2),p.R_w,[0.18 0.18 0.18]);
    draw_wheel(ax, Wm(1),Wm(2),p.R_w,[0.18 0.18 0.18]);
    draw_wheel(ax, Wr(1),Wr(2),p.R_w,[0.18 0.18 0.18]);
end

%% ═════════════════════════════════════════════════════════════
%% 헬퍼: 정보 텍스트 빌더
%% ═════════════════════════════════════════════════════════════
function s = build_info_str(p)
    s1 = sprintf('[Rocker: %s]\n', upper(p.rocker_mode));
    if strcmp(p.rocker_mode, 'frame')
        s1 = [s1 sprintf('T_r = %.0f mm\nS_r1 = %.0f mm\nS_r2 = %.0f mm\n', p.T_r*1000, p.S_r1*1000, p.S_r2*1000)];
    else
        s1 = [s1 sprintf('L_r1 = %.0f mm\nL_r2 = %.0f mm\n', p.L_r1*1000, p.L_r2*1000)];
    end
    
    s2 = sprintf('\n[Bogie: %s]\n', upper(p.bogie_mode));
    if strcmp(p.bogie_mode, 'frame')
        s2 = [s2 sprintf('T_b = %.0f mm\nS_b1 = %.0f mm\nS_b2 = %.0f mm', p.T_b*1000, p.S_b1*1000, p.S_b2*1000)];
    else
        s2 = [s2 sprintf('L_b1 = %.0f mm\nL_b2 = %.0f mm', p.L_b1*1000, p.L_b2*1000)];
    end
    s = [s1 s2];
end

%% ═════════════════════════════════════════════════════════════
%% 헬퍼: 거리 산출용 로컬 함수 (ZETIN_JointOptSearch_v3.m 호환)
%% ═════════════════════════════════════════════════════════════
function cb = get_cb_fwd_local(p)
    switch lower(p.bogie_mode)
        case 'triangle', cb = p.L_b1 * abs(sin(p.beta_b/2));
        case 'frame',    cb = p.S_b1 * abs(cos(p.th_b1));
        otherwise,       cb = p.c_b;
    end
    cb = max(cb, p.R_w);
end
function b = get_b_eff_local(p)
    switch lower(p.rocker_mode)
        case 'triangle', b = p.L_r2 * abs(cos(p.alpha_r/2));
        case 'frame',    b = p.j_r*p.T_r + p.S_r2*abs(sin(p.th_r2));
        otherwise,       b = p.b_r;
    end
    b = max(b, p.R_w);
end
function a = get_a_eff_local(p)
    switch lower(p.rocker_mode)
        case 'triangle', a = p.L_r1 * abs(cos(p.alpha_r/2));
        case 'frame',    a = (1-p.j_r)*p.T_r + p.S_r1*abs(sin(p.th_r1));
        otherwise,       a = p.a_r;
    end
    a = max(a, p.R_w);
end