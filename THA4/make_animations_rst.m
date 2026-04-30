%% make_animations_rst.m — THA4 RST Robot Animations
%  Creates full-mesh KR120 animations for:
%    ANIM 1: Cfg4 shaft stabilisation — mode (a) vs (b) side by side
%    ANIM 2: Cfg5 virtual wall deflection — mode (a) robot + both trajectories
%
%  Requires Robotics System Toolbox (importrobot, show).
%  AVIs saved to THA4/figures/ alongside the static PNGs.

clear; close all;
addpath(genpath(fileparts(mfilename('fullpath')) + "/../"));

fig_dir = fullfile(fileparts(mfilename('fullpath')), 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

fprintf('Loading robot params and RST model...\n');
robot = KR120_params();
rbt   = load_kr120_rst();

L_tool = 0.1;

%% =========================================================================
%  SHARED QP OPTS  (identical to test_THA4)
%  =========================================================================
opts_a.L_tool = L_tool;  opts_a.d_max  = 0.003;
opts_a.lambda = 1e-4;    opts_a.mu     = 0;
opts_a.step   = 0.02;    opts_a.dq_max = 0.05;
opts_b = opts_a;  opts_b.mu = 0.01;

%% =========================================================================
%  RUN SCENARIOS
%  =========================================================================
fprintf('Running Cfg4 (shaft stabilisation, mode a+b)...\n');
q0_4   = [0;-0.4;0.5;0;0.3;0];
goal_4 = [-0.5; 1.5; 1.2];
[qa_4, pa_4] = simulate_VF(robot, q0_4, goal_4, 600, opts_a);
[qb_4, pb_4] = simulate_VF(robot, q0_4, goal_4, 600, opts_b);

fprintf('Running Cfg5 (wall blocks goal, mode a+b)...\n');
wall5.n = [0;-1;0];  wall5.p = [1.9; 0.20; 0.93];  wall5.margin = 0.02;
oa5 = opts_a;  oa5.wall = wall5;
ob5 = opts_b;  ob5.wall = wall5;
q0_5   = [0;-0.4;0.5;0;0.3;0];
goal_5 = [1.8;0.4;1.0];
[qa_5, pa_5] = simulate_VF(robot, q0_5, goal_5, 600, oa5);
[qb_5, pb_5] = simulate_VF(robot, q0_5, goal_5, 600, ob5);

%% =========================================================================
%  ANIM 1 RST — Cfg4 shaft stabilisation: mode (a) left vs mode (b) right
%  =========================================================================
K1   = min(size(pa_4,2), size(pb_4,2));
XL1  = [-2.0  2.5];  YL1 = [-1.0  2.5];  ZL1 = [-0.3  3.0];
skip = 2;

hf1  = figure('Color','w','Position',[80 80 1200 600]);
figure(hf1);
ax1L = subplot(1,2,1);
ax1R = subplot(1,2,2);

v1 = VideoWriter(fullfile(fig_dir,'anim1_rst_shaft_cfg4.avi'),'Motion JPEG AVI');
v1.FrameRate = 15;  v1.Quality = 90;  open(v1);
fprintf('Rendering ANIM 1 RST (%d frames)...\n', ceil(K1/skip));

for k = 1:skip:K1
    % --- Left panel: mode (a) ---
    rst_frame(hf1, ax1L, rbt, qa_4(:,k), XL1, YL1, ZL1, [1 1 1]);
    hold(ax1L,'on');
    plot3(ax1L, pa_4(1,1:k), pa_4(2,1:k), pa_4(3,1:k), 'b-', 'LineWidth',1.5);
    draw_tool(ax1L, robot, qa_4(:,k), L_tool, 'b');
    plot3(ax1L, goal_4(1),goal_4(2),goal_4(3), 'k*','MarkerSize',10);
    title(ax1L, sprintf('Mode (a)  step %d',k), 'Color','k');

    % --- Right panel: mode (b) ---
    rst_frame(hf1, ax1R, rbt, qb_4(:,k), XL1, YL1, ZL1, [1 1 1]);
    hold(ax1R,'on');
    plot3(ax1R, pb_4(1,1:k), pb_4(2,1:k), pb_4(3,1:k), 'r-', 'LineWidth',1.5);
    draw_tool(ax1R, robot, qb_4(:,k), L_tool, 'r');
    plot3(ax1R, goal_4(1),goal_4(2),goal_4(3), 'k*','MarkerSize',10);
    title(ax1R, sprintf('Mode (b)  step %d',k), 'Color','k');

    drawnow;
    writeVideo(v1, getframe(hf1));
end
close(v1);
fprintf('Saved: anim1_rst_shaft_cfg4.avi\n');

%% =========================================================================
%  ANIM 2 RST — Cfg5 virtual wall: mode (a) robot + both trajectories
%  =========================================================================
K2  = min(size(pa_5,2), size(pb_5,2));
XL2 = [-0.5  3.0];  YL2 = [-1.5  2.0];  ZL2 = [-0.3  3.0];

wp5 = wall5.p;
xr5 = linspace(wp5(1)-0.5, wp5(1)+0.5, 2);
zr5 = linspace(wp5(3)-0.3, wp5(3)+0.3, 2);
[xw5, zw5] = meshgrid(xr5, zr5);

hf2  = figure('Color','w','Position',[80 80 900 700]);
figure(hf2);
ax2  = axes('Parent', hf2);

v2 = VideoWriter(fullfile(fig_dir,'anim2_rst_wall_cfg5.avi'),'Motion JPEG AVI');
v2.FrameRate = 15;  v2.Quality = 90;  open(v2);
fprintf('Rendering ANIM 2 RST (%d frames)...\n', ceil(K2/skip));

for k = 1:skip:K2
    % Robot shown at mode (a) config; both trajectories overlaid as lines
    rst_frame(hf2, ax2, rbt, qa_5(:,k), XL2, YL2, ZL2, [1 -0.5 1.5]);
    hold(ax2,'on');

    % Wall plane
    surf(ax2, xw5, wp5(2)*ones(size(xw5)), zw5, 'FaceAlpha',0.25, ...
        'FaceColor',[0.85 0.33 0.10],'EdgeColor','none','HandleVisibility','off');

    % Trajectories (blue solid = a, red dashed = b)
    plot3(ax2, pa_5(1,1:k), pa_5(2,1:k), pa_5(3,1:k), 'b-',  'LineWidth',2,'HandleVisibility','off');
    plot3(ax2, pb_5(1,1:k), pb_5(2,1:k), pb_5(3,1:k), 'r--', 'LineWidth',2,'HandleVisibility','off');

    % Moving tip dots
    plot3(ax2, pa_5(1,k), pa_5(2,k), pa_5(3,k), 'bo','MarkerSize',8,'MarkerFaceColor','b','HandleVisibility','off');
    plot3(ax2, pb_5(1,k), pb_5(2,k), pb_5(3,k), 'ro','MarkerSize',8,'MarkerFaceColor','r','HandleVisibility','off');

    % Goal
    plot3(ax2, goal_5(1),goal_5(2),goal_5(3), 'k*','MarkerSize',10,'HandleVisibility','off');

    title(ax2, sprintf('Cfg5 — Virtual Wall  (blue=a  red--=b)  step %d',k), 'Color','k');
    drawnow;
    writeVideo(v2, getframe(hf2));
end
close(v2);
fprintf('Saved: anim2_rst_wall_cfg5.avi\n');

%% =========================================================================
%  LOCAL HELPERS
%  =========================================================================

function rst_frame(hfig, ax, rbt, q, XL, YL, ZL, vw)
% Clear axes, render RST robot mesh, restore axes settings.
% vw: optional view direction passed to view(ax, vw). Default [-35 25].
% Follows the same pattern as robot_animation_rst.m from THA2.

    if nargin < 8,  vw = [-35 25];  end

    % Clear axes children but preserve axis labels
    kids = findall(ax);
    keep = [ax; ax.XLabel; ax.YLabel; ax.ZLabel];
    delete(kids(~ismember(kids, keep)));
    hold(ax,'on');
    camlight(ax,'headlight');

    % Render robot mesh at configuration q
    show(rbt, q(:)', 'Parent',ax, 'PreservePlot',true, 'Visuals','on', 'Frames','off');

    % show() resets view, limits and colours — restore them
    set(ax,'Color','w','XColor','k','YColor','k','ZColor','k', ...
           'GridColor',[0.82 0.82 0.82]);
    grid(ax,'on');
    view(ax, vw);
    xlim(ax,XL);  ylim(ax,YL);  zlim(ax,ZL);
    xlabel(ax,'X (m)');  ylabel(ax,'Y (m)');  zlabel(ax,'Z (m)');

    % Close any stray figures opened by show(), restore current figure
    all_figs = findall(0,'Type','figure');
    for fj = 1:numel(all_figs)
        if all_figs(fj) ~= hfig,  close(all_figs(fj));  end
    end
    set(0,'CurrentFigure',hfig);
end

function draw_tool(ax, robot, q, L_tool, color)
% Draw the 100 mm cylindrical tool as a thick coloured line from EE to tip,
% extended slightly for visibility in the large-workspace 3D view.

    T_ee  = FK_space(robot.M, robot.Slist, q);
    p_ee  = T_ee(1:3,4);
    d     = T_ee(1:3,1:3) * [0;0;1];
    p_tip = p_ee + d * L_tool;

    % Draw from 50 mm behind EE to 50 mm past tip for visual clarity
    p_a = p_ee  - d * 0.05;
    p_b = p_tip + d * 0.05;
    plot3(ax, [p_a(1) p_b(1)], [p_a(2) p_b(2)], [p_a(3) p_b(3)], ...
        '-', 'Color',color, 'LineWidth',4);
end
