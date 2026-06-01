function plot_sim(x_arr, x_t, y_t, R, p, label, col)
    hold on;

    fill([x_t, fliplr(x_t)], ...
         [y_t, -0.06*ones(1,length(y_t))], ...
         [0.78 0.68 0.58], 'EdgeColor','none', 'FaceAlpha',0.5);
    plot(x_t, y_t, 'k-', 'LineWidth', 1.2, 'HandleVisibility', 'off');

    % 스냅샷 로봇 자세 (HandleVisibility off → 범례 제외)
    f_idx = round(linspace(15, length(x_arr)-15, 8));
    for fi = f_idx
        draw_robot_snap(x_arr(fi), R, fi, p);
    end

    % 범례에 포함할 선만 따로 plot
    h1 = plot(x_arr, R.y0, '-', 'Color', col, 'LineWidth', 2.2);
    
    y_wr = interp1(x_t, y_t, R.xwr, 'linear', 'extrap');
    y_wm = interp1(x_t, y_t, R.xwm, 'linear', 'extrap');
    y_wf = interp1(x_t, y_t, R.xwf, 'linear', 'extrap');
    y_avg = (y_wr + y_wm + y_wf) / 3 + p.R_w;
    h_offset = mean(R.y0 - y_avg, 'omitnan');
    y_ideal = y_avg + h_offset;
    
    h2 = plot(x_arr, y_ideal, '--', 'Color', col*0.65, 'LineWidth', 1.0);

    legend([h1 h2], {'차체 높이 (P0)', '이상 궤적 (평활화)'}, ...
           'Location', 'northwest', 'FontSize', 7);

    grid on; box on;
    xlabel('X [m]', 'FontSize', 8);
    ylabel('Y [m]', 'FontSize', 8);
    title(label, 'FontSize', 9, 'FontWeight', 'bold');
    xlim([x_t(1), x_t(end)]);
    ylim([-0.05, max(y_t) + p.R_w*3 + p.h_body + 0.1]);
end

function draw_robot_snap(xb, R, i, p)
    % 모든 핸들에 HandleVisibility='off' 적용 → 범례에 안 잡힘
    X = [R.y0(i); R.ar(i); R.bb(i)];
    [Wf, Wm, Wr, Pb, ~] = wpos(X, xb, p);   % v2: 5출력 (CG 무시)
    P0 = [xb; R.y0(i)];

    hv = {'HandleVisibility','off'};
    plot([Wr(1),P0(1),Pb(1)],[Wr(2),P0(2),Pb(2)],'b-','LineWidth',2.0, hv{:});
    plot([Wm(1),Pb(1),Wf(1)],[Wm(2),Pb(2),Wf(2)],'g-','LineWidth',2.0, hv{:});

    bw=0.07; bh=p.h_body;
    fill([P0(1)-bw,P0(1)+bw,P0(1)+bw,P0(1)-bw], ...
         [P0(2),P0(2),P0(2)+bh,P0(2)+bh], ...
         [0.4 0.6 0.85],'FaceAlpha',0.55,'EdgeColor',[0.1 0.3 0.7], hv{:});

    plot(P0(1),P0(2),'bs','MarkerSize',5,'MarkerFaceColor','b', hv{:});
    plot(Pb(1),Pb(2),'gs','MarkerSize',4,'MarkerFaceColor','g', hv{:});

    th = linspace(0,2*pi,30);
    for W = {Wf, Wm, Wr}
        w = W{1};
        fill(w(1)+p.R_w*cos(th), w(2)+p.R_w*sin(th), ...
             [0.18 0.18 0.18],'FaceAlpha',0.72,'EdgeColor','none', hv{:});
    end
end