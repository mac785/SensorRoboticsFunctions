%% make_animations_rst.m — THA4 RST Robot Animations
%  Creates full-mesh KR120 animations for:
%    ANIM 1: Cfg4 shaft stabilisation — mode (a) vs (b) side by side
%    ANIM 2: Cfg5 virtual wall — wall blocks goal
%    ANIM 3: Cfg6 virtual wall — wall as approach limiter (goal accessible)
%
%  Requires Robotics System Toolbox (importrobot, show).
%  Output saved to THA4/figures/.

clear; close all;
addpath(genpath(fileparts(mfilename('fullpath')) + "/../"));

% ---- TOGGLES ---------------------------------------------------------------
OUTPUT_FORMAT = 'gif';      % 'avi'  or  'gif'
RENDER_ANIMS  = 3;          % which animations to render: 1=Cfg4, 2=Cfg5, 3=Cfg6 (e.g. [1 3] for both)
% ----------------------------------------------------------------------------

fig_dir = fullfile(fileparts(mfilename('fullpath')), 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

fprintf('Loading robot params and RST model...\n');
robot = KR120_params();
rbt   = load_kr120_rst();

L_tool = 0.1;
skip   = 2;
fps    = 15;

%% =========================================================================
%  SHARED QP OPTS  (identical to test_THA4)
%  =========================================================================
opts_a.L_tool = L_tool;  opts_a.d_max  = 0.003;
opts_a.lambda = 1e-4;    opts_a.mu     = 0;
opts_a.step   = 0.02;    opts_a.dq_max = 0.05;
opts_b = opts_a;  opts_b.mu = 0.01;

%% =========================================================================
%  RUN SIMULATIONS (only for requested animations)
%  =========================================================================
if any(RENDER_ANIMS == 1)
    fprintf('Running Cfg4 (shaft stabilisation, mode a+b)...\n');
    q0_4   = [0;-0.4;0.5;0;0.3;0];
    goal_4 = [-0.5; 1.5; 1.2];
    [qa_4, pa_4] = simulate_VF(robot, q0_4, goal_4, 600, opts_a);
    [qb_4, pb_4] = simulate_VF(robot, q0_4, goal_4, 600, opts_b);
end

if any(RENDER_ANIMS == 2)
    fprintf('Running Cfg5 (wall blocks goal, mode a+b)...\n');
    wall5.n = [0;-1;0];  wall5.p = [1.9; 0.20; 0.93];  wall5.margin = 0.02;
    oa5 = opts_a;  oa5.wall = wall5;
    ob5 = opts_b;  ob5.wall = wall5;
    q0_5   = [0;-0.4;0.5;0;0.3;0];
    goal_5 = [1.8;0.4;1.0];
    [qa_5, pa_5] = simulate_VF(robot, q0_5, goal_5, 600, oa5);
    [qb_5, pb_5] = simulate_VF(robot, q0_5, goal_5, 600, ob5);
end

if any(RENDER_ANIMS == 3)
    fprintf('Running Cfg6 (wall as approach limiter, mode a+b)...\n');
    wall6.n = [0;-1;0];  wall6.p = [1.9; 0.35; 0.95];  wall6.margin = 0.02;
    oa6 = opts_a;  oa6.wall = wall6;
    ob6 = opts_b;  ob6.wall = wall6;
    q0_6   = [0;-0.4;0.5;0;0.3;0];
    goal_6 = [1.8;0.25;1.0];
    [qa_6, pa_6] = simulate_VF(robot, q0_6, goal_6, 600, oa6);
    [qb_6, pb_6] = simulate_VF(robot, q0_6, goal_6, 600, ob6);
end

%% =========================================================================
%  ANIM 1 — Cfg4 shaft stabilisation: mode (a) left vs mode (b) right
%  =========================================================================
if any(RENDER_ANIMS == 1)
    K1  = min(size(pa_4,2), size(pb_4,2));
    XL1 = [-2.0  2.5];  YL1 = [-1.0  2.5];  ZL1 = [-0.3  3.0];

    hf1  = figure('Color','w','Position',[80 80 1200 600]);
    ax1L = subplot(1,2,1);
    ax1R = subplot(1,2,2);

    fname1 = fullfile(fig_dir, ['anim1_rst_shaft_cfg4.' OUTPUT_FORMAT]);
    wr1 = writer_open(fname1, OUTPUT_FORMAT, fps);
    fprintf('Rendering ANIM 1 (%d frames) → %s...\n', ceil(K1/skip), OUTPUT_FORMAT);

    for k = 1:skip:K1
        rst_frame(hf1, ax1L, rbt, qa_4(:,k), XL1, YL1, ZL1, [1 1 1]);
        hold(ax1L,'on');
        plot3(ax1L, pa_4(1,1:k), pa_4(2,1:k), pa_4(3,1:k), 'b-', 'LineWidth',1.5);
        draw_tool(ax1L, robot, qa_4(:,k), L_tool, 'b');
        plot3(ax1L, goal_4(1),goal_4(2),goal_4(3), 'k*','MarkerSize',10);
        title(ax1L, sprintf('Mode (a)  step %d',k), 'Color','k');

        rst_frame(hf1, ax1R, rbt, qb_4(:,k), XL1, YL1, ZL1, [1 1 1]);
        hold(ax1R,'on');
        plot3(ax1R, pb_4(1,1:k), pb_4(2,1:k), pb_4(3,1:k), 'r-', 'LineWidth',1.5);
        draw_tool(ax1R, robot, qb_4(:,k), L_tool, 'r');
        plot3(ax1R, goal_4(1),goal_4(2),goal_4(3), 'k*','MarkerSize',10);
        title(ax1R, sprintf('Mode (b)  step %d',k), 'Color','k');

        drawnow;
        wr1 = writer_frame(wr1, hf1);
    end
    writer_close(wr1);
    fprintf('Saved: anim1_rst_shaft_cfg4.%s\n', OUTPUT_FORMAT);
end

%% =========================================================================
%  ANIM 2 — Cfg5 virtual wall: wall blocks goal
%  =========================================================================
if any(RENDER_ANIMS == 2)
    K2  = min(size(pa_5,2), size(pb_5,2));
    XL2 = [-0.5  3.0];  YL2 = [-1.5  2.0];  ZL2 = [-0.3  3.0];

    wp5 = wall5.p;
    xr5 = linspace(wp5(1)-0.5, wp5(1)+0.5, 2);
    zr5 = linspace(wp5(3)-0.3, wp5(3)+0.3, 2);
    [xw5, zw5] = meshgrid(xr5, zr5);

    hf2 = figure('Color','w','Position',[80 80 900 700]);
    ax2 = axes('Parent', hf2);

    fname2 = fullfile(fig_dir, ['anim2_rst_wall_cfg5.' OUTPUT_FORMAT]);
    wr2 = writer_open(fname2, OUTPUT_FORMAT, fps);
    fprintf('Rendering ANIM 2 (%d frames) → %s...\n', ceil(K2/skip), OUTPUT_FORMAT);

    for k = 1:skip:K2
        rst_frame(hf2, ax2, rbt, qa_5(:,k), XL2, YL2, ZL2, [1 -0.5 1.5]);
        hold(ax2,'on');
        surf(ax2, xw5, wp5(2)*ones(size(xw5)), zw5, 'FaceAlpha',0.25, ...
            'FaceColor',[0.85 0.33 0.10],'EdgeColor','none','HandleVisibility','off');
        plot3(ax2, pa_5(1,1:k), pa_5(2,1:k), pa_5(3,1:k), 'b-',  'LineWidth',2,'HandleVisibility','off');
        plot3(ax2, pb_5(1,1:k), pb_5(2,1:k), pb_5(3,1:k), 'r--', 'LineWidth',2,'HandleVisibility','off');
        plot3(ax2, pa_5(1,k), pa_5(2,k), pa_5(3,k), 'bo','MarkerSize',8,'MarkerFaceColor','b','HandleVisibility','off');
        plot3(ax2, pb_5(1,k), pb_5(2,k), pb_5(3,k), 'ro','MarkerSize',8,'MarkerFaceColor','r','HandleVisibility','off');
        plot3(ax2, goal_5(1),goal_5(2),goal_5(3), 'k*','MarkerSize',10,'HandleVisibility','off');
        title(ax2, sprintf('Cfg5 — Wall Blocks Goal  (blue=a  red--=b)  step %d',k), 'Color','k');
        drawnow;
        wr2 = writer_frame(wr2, hf2);
    end
    writer_close(wr2);
    fprintf('Saved: anim2_rst_wall_cfg5.%s\n', OUTPUT_FORMAT);
end

%% =========================================================================
%  ANIM 3 — Cfg6 virtual wall: goal accessible, wall limits overshoot
%  Camera mirrored across XZ plane vs ANIM 2 (view from +Y side)
%  =========================================================================
if any(RENDER_ANIMS == 3)
    K3  = min(size(pa_6,2), size(pb_6,2));
    XL3 = [0.5  3.0];  YL3 = [-0.8  1.5];  ZL3 = [0.0  2.5];

    wp6 = wall6.p;
    xr6 = linspace(wp6(1)-0.5, wp6(1)+0.5, 2);
    zr6 = linspace(wp6(3)-0.4, wp6(3)+0.4, 2);
    [xw6, zw6] = meshgrid(xr6, zr6);

    hf3 = figure('Color','w','Position',[80 80 900 700]);
    ax3 = axes('Parent', hf3);

    fname3 = fullfile(fig_dir, ['anim3_rst_wall_cfg6.' OUTPUT_FORMAT]);
    wr3 = writer_open(fname3, OUTPUT_FORMAT, fps);
    fprintf('Rendering ANIM 3 (%d frames) → %s...\n', ceil(K3/skip), OUTPUT_FORMAT);

    for k = 1:skip:K3
        rst_frame(hf3, ax3, rbt, qa_6(:,k), XL3, YL3, ZL3, [1 0.5 1.5]);
        hold(ax3,'on');
        surf(ax3, xw6, wp6(2)*ones(size(xw6)), zw6, 'FaceAlpha',0.25, ...
            'FaceColor',[0.85 0.33 0.10],'EdgeColor','none','HandleVisibility','off');
        plot3(ax3, pa_6(1,1:k), pa_6(2,1:k), pa_6(3,1:k), 'b-',  'LineWidth',2,'HandleVisibility','off');
        plot3(ax3, pb_6(1,1:k), pb_6(2,1:k), pb_6(3,1:k), 'r--', 'LineWidth',2,'HandleVisibility','off');
        plot3(ax3, pa_6(1,k), pa_6(2,k), pa_6(3,k), 'bo','MarkerSize',8,'MarkerFaceColor','b','HandleVisibility','off');
        plot3(ax3, pb_6(1,k), pb_6(2,k), pb_6(3,k), 'ro','MarkerSize',8,'MarkerFaceColor','r','HandleVisibility','off');
        plot3(ax3, goal_6(1),goal_6(2),goal_6(3), 'k*','MarkerSize',10,'HandleVisibility','off');
        title(ax3, sprintf('Cfg6 — Wall as Limiter  (blue=a  red--=b)  step %d',k), 'Color','k');
        drawnow;
        wr3 = writer_frame(wr3, hf3);
    end
    writer_close(wr3);
    fprintf('Saved: anim3_rst_wall_cfg6.%s\n', OUTPUT_FORMAT);
end

%% =========================================================================
%  LOCAL HELPERS
%  =========================================================================

function wr = writer_open(path, fmt, fps)
    wr.path = path;
    wr.fmt  = fmt;
    wr.fps  = fps;
    wr.n    = 0;
    if strcmp(fmt, 'avi')
        wr.v = VideoWriter(path, 'Motion JPEG AVI');
        wr.v.FrameRate = fps;
        wr.v.Quality   = 90;
        open(wr.v);
    else
        wr.v = [];
    end
end

function wr = writer_frame(wr, hfig)
    wr.n = wr.n + 1;
    frame = getframe(hfig);
    if strcmp(wr.fmt, 'avi')
        writeVideo(wr.v, frame);
    else
        img = frame2im(frame);
        [imind, cm] = rgb2ind(img, 256);
        if wr.n == 1
            imwrite(imind, cm, wr.path, 'gif', 'Loopcount', inf, 'DelayTime', 1/wr.fps);
        else
            imwrite(imind, cm, wr.path, 'gif', 'WriteMode', 'append', 'DelayTime', 1/wr.fps);
        end
    end
end

function writer_close(wr)
    if strcmp(wr.fmt, 'avi')
        close(wr.v);
    end
end

function rst_frame(hfig, ax, rbt, q, XL, YL, ZL, vw)
    if nargin < 8,  vw = [-35 25];  end
    kids = findall(ax);
    keep = [ax; ax.XLabel; ax.YLabel; ax.ZLabel];
    delete(kids(~ismember(kids, keep)));
    hold(ax,'on');
    camlight(ax,'headlight');
    show(rbt, q(:)', 'Parent',ax, 'PreservePlot',true, 'Visuals','on', 'Frames','off');
    set(ax,'Color','w','XColor','k','YColor','k','ZColor','k', ...
           'GridColor',[0.82 0.82 0.82]);
    grid(ax,'on');
    view(ax, vw);
    xlim(ax,XL);  ylim(ax,YL);  zlim(ax,ZL);
    xlabel(ax,'X (m)');  ylabel(ax,'Y (m)');  zlabel(ax,'Z (m)');
    all_figs = findall(0,'Type','figure');
    for fj = 1:numel(all_figs)
        if all_figs(fj) ~= hfig,  close(all_figs(fj));  end
    end
    set(0,'CurrentFigure',hfig);
end

function draw_tool(ax, robot, q, L_tool, color)
    T_ee  = FK_space(robot.M, robot.Slist, q);
    p_ee  = T_ee(1:3,4);
    d     = T_ee(1:3,1:3) * [0;0;1];
    p_tip = p_ee + d * L_tool;
    p_a   = p_ee  - d * 0.05;
    p_b   = p_tip + d * 0.05;
    plot3(ax, [p_a(1) p_b(1)], [p_a(2) p_b(2)], [p_a(3) p_b(3)], ...
        '-', 'Color',color, 'LineWidth',4);
end
