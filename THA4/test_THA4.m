%% test_THA4.m — ME384R THA4 Programming Assignment
%  KUKA KR120 R2500 Pro (Quantec Nano) with 100 mm cylindrical tool.
%
%  Part (a): approach p_goal with joint limits + 3 mm sphere constraint
%  Part (b): same + penalise change in tool shaft direction
%  Part (c): parts (a) and (b) repeated with a planar virtual wall
%  Part (d): comparison plots across all parts / four configurations
%
%  Scenarios are defined as a struct array so adding a new test case is a
%  one-line addition.  Each scenario is run twice (mode a, mode b) and the
%  results are stored in a cell array indexed by scenario number.

clear; close all;
addpath(genpath(fileparts(mfilename('fullpath')) + "/../"));

% ---- TOGGLES ----
SHOW_FIGURES   = false;   % false → figures render offscreen and save as PNG only
RUN_ANIMATIONS = true;   % false → skip MP4 rendering entirely
% -----------------

fig_dir = fullfile(fileparts(mfilename('fullpath')), 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

% Force light theme for all figures regardless of MATLAB preference.
% GridColor must be set explicitly — dark-mode default is near-white,
% which becomes invisible once the axes background is forced to white.
set(groot, 'defaultFigureColor',    'w', ...
           'defaultAxesColor',      'w', ...
           'defaultAxesXColor',     'k', ...
           'defaultAxesYColor',     'k', ...
           'defaultAxesZColor',     'k', ...
           'defaultTextColor',      'k', ...
           'defaultAxesGridColor',  [0.15 0.15 0.15], ...
           'defaultAxesGridAlpha',  0.15, ...
           'defaultLegendColor',    'w', ...
           'defaultLegendTextColor','k', ...
           'defaultLegendEdgeColor',[0.5 0.5 0.5]);

if ~SHOW_FIGURES
    set(groot, 'defaultFigureVisible', 'off');
end

robot = KR120_params();

%% =========================================================================
%  SHARED PARAMETERS
%  =========================================================================
L_tool = 0.1;    % 100 mm tool length (m)
d_max  = 0.003;  % 3 mm sphere constraint radius (m)

opts_a.L_tool = L_tool;
opts_a.d_max  = d_max;
opts_a.lambda = 1e-4;
opts_a.mu     = 0;
opts_a.step   = 0.02;
opts_a.dq_max = 0.05;

opts_b      = opts_a;
opts_b.mu   = 0.01;

N_MAX = 600;

%% =========================================================================
%  SCENARIO DEFINITIONS
%  =========================================================================
% Each scenario:  label, q0, p_goal, wall (or []), description
sc = struct('label',{},'q0',{},'p_goal',{},'wall',{},'desc',{});

% --- Cfg 1: baseline, mid-workspace ---
sc(1) = scen('Cfg1', [0;-0.4;0.5;0;0.3;0], [1.8;0.4;1.0], [], ...
    'Baseline: bent arm, mid-workspace goal, no special constraints.');

% --- Cfg 2: different goal direction (negative Y, higher Z) ---
sc(2) = scen('Cfg2', [0;-0.4;0.5;0;0.3;0], [2.1;-0.3;1.2], [], ...
    'Different goal direction (-Y, higher Z) from same start.');

% --- Cfg 3: joint-limit pressure (J2 driven to upper limit +0.611 rad) ---
sc(3) = scen('Cfg3', [0; 0.0; 0.3; 0; 0.0; 0], [1.5; 0.0; -1.5], [], ...
    'Goal far below base; only reachable past J2 upper limit. Tests joint-limit constraint.');

% --- Cfg 4: reorientation-heavy (goal behind+across, ~60 deg shaft swing) ---
sc(4) = scen('Cfg4', [0;-0.4;0.5;0;0.3;0], [-0.5; 1.5; 1.2], [], ...
    'Goal behind robot — natural path requires large wrist reorientation. Tests part (b) shaft stabilisation.');

% --- Cfg 5: virtual wall blocks goal (Wall 1 case) ---
wall1.n = [0;-1;0]; wall1.p = [1.9; 0.20; 0.93]; wall1.margin = 0.02;
sc(5) = scen('Cfg5-Wall1', [0;-0.4;0.5;0;0.3;0], [1.8;0.4;1.0], wall1, ...
    'Wall at Y=0.20 between robot and goal at Y=0.40 — wall blocks direct approach.');

% --- Cfg 6: wall as approach-limiter, goal accessible (Wall 2 case) ---
wall2.n = [0;-1;0]; wall2.p = [1.9; 0.35; 0.95]; wall2.margin = 0.02;
sc(6) = scen('Cfg6-Wall2', [0;-0.4;0.5;0;0.3;0], [1.8;0.25;1.0], wall2, ...
    'Wall at Y=0.35 with goal at Y=0.25 (accessible) — wall constrains approach overshoot.');

%% =========================================================================
%  RUN ALL SCENARIOS
%  =========================================================================
fprintf('========================================================\n');
fprintf('  Running %d scenarios x 2 modes\n', length(sc));
fprintf('========================================================\n');

results = cell(length(sc), 1);
for s = 1:length(sc)
    fprintf('Scenario %d (%s)...\n', s, sc(s).label);

    p_start = tool_tip_fk(robot, sc(s).q0);
    fprintf('  start tip [%.3f %.3f %.3f] | goal [%.3f %.3f %.3f] | dist %.1f mm\n', ...
        p_start(1),p_start(2),p_start(3), sc(s).p_goal(1),sc(s).p_goal(2),sc(s).p_goal(3), ...
        norm(sc(s).p_goal - p_start)*1e3);

    oa = opts_a;
    ob = opts_b;
    if ~isempty(sc(s).wall)
        oa.wall = sc(s).wall;
        ob.wall = sc(s).wall;
    end

    [qa, pa, ia] = simulate_VF(robot, sc(s).q0, sc(s).p_goal, N_MAX, oa);
    [qb, pb, ib] = simulate_VF(robot, sc(s).q0, sc(s).p_goal, N_MAX, ob);

    results{s}.a = struct('q', qa, 'p', pa, 'info', ia);
    results{s}.b = struct('q', qb, 'p', pb, 'info', ib);
end

%% =========================================================================
%  SUMMARY TABLE
%  =========================================================================
fprintf('\n========================================================\n');
fprintf('  RESULTS SUMMARY\n');
fprintf('========================================================\n');
fprintf('%-14s | Mode | Steps | FinalDist | ShaftSwing | JointHit\n', 'Scenario');
fprintf('%s\n', repmat('-',1,72));
ref = [0;0;1];
for s = 1:length(sc)
    for mode = ['a','b']
        r = results{s}.(mode);
        dist_mm = arrayfun(@(t) t.dist, r.info) * 1e3;
        swing = shaft_swing(robot, r.q, ref);
        hits  = joint_limit_hits(r.q, robot.joint_limits);
        if isempty(hits), hit_str = '-'; else, hit_str = strjoin(hits, ','); end
        fprintf('%-14s |  (%s) | %5d | %8.4f | %9.2f° | %s\n', ...
            sc(s).label, mode, size(r.p,2)-1, dist_mm(end), swing, hit_str);
    end
end

%% =========================================================================
%  FIG 1 — Master 3D trajectories (tiled, one per scenario)
%  =========================================================================
figure('Name','Fig1: 3D Trajectories','Color','w','Position',[80 80 1300 800]);
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
for s = 1:length(sc)
    nexttile; hold on; grid on; axis equal; view([-35 25]);
    pa = results{s}.a.p; pb = results{s}.b.p;
    plot3(pa(1,:), pa(2,:), pa(3,:), 'b-',  'LineWidth',1.5,'DisplayName','(a)');
    plot3(pb(1,:), pb(2,:), pb(3,:), 'r--', 'LineWidth',1.5,'DisplayName','(b)');
    plot3(sc(s).p_goal(1), sc(s).p_goal(2), sc(s).p_goal(3), 'k*', ...
        'MarkerSize',10,'HandleVisibility','off');
    p0 = tool_tip_fk(robot, sc(s).q0);
    plot3(p0(1),p0(2),p0(3),'ko','MarkerSize',6,'MarkerFaceColor','k','HandleVisibility','off');
    if ~isempty(sc(s).wall)
        % Draw wall plane (small patch around the goal)
        wp = sc(s).wall.p;
        if abs(sc(s).wall.n(2)) > 0.5    % wall normal in Y direction
            xr = linspace(wp(1)-0.5, wp(1)+0.5, 2);
            zr = linspace(wp(3)-0.3, wp(3)+0.3, 2);
            [xw,zw] = meshgrid(xr,zr);
            surf(xw, wp(2)*ones(size(xw)), zw, 'FaceAlpha',0.2, ...
                'FaceColor',[0.3 0.8 0.8],'EdgeColor','none','HandleVisibility','off');
        end
    end
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(sprintf('%s', sc(s).label));
    if s == 1, legend('Location','best'); end
    hold off;
end
sgtitle('3D Tool Tip Trajectories — All Scenarios (a) vs (b)', 'Color', 'k');
exportgraphics(gcf, fullfile(fig_dir, 'fig1_3d_trajectories.png'), 'Resolution', 150);

%% =========================================================================
%  FIG 2 — Distance to goal vs step (tiled)
%  =========================================================================
figure('Name','Fig2: Distance vs Step','Color','w','Position',[80 80 1300 700]);
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
for s = 1:length(sc)
    nexttile; hold on; grid on;
    da = arrayfun(@(t) t.dist, results{s}.a.info) * 1e3;
    db = arrayfun(@(t) t.dist, results{s}.b.info) * 1e3;
    plot(da, 'b-',  'LineWidth',1.5, 'DisplayName','(a)');
    plot(db, 'r--', 'LineWidth',1.5, 'DisplayName','(b)');
    yline(d_max*1e3,'k:','3 mm','LabelHorizontalAlignment','left');
    xlabel('Step'); ylabel('Dist to goal (mm)');
    title(sc(s).label);
    set(gca,'YScale','log');
    if s == 1, legend('Location','southwest'); end
    hold off;
end
sgtitle('Distance to Goal vs. Step — All Scenarios', 'Color', 'k');
exportgraphics(gcf, fullfile(fig_dir, 'fig2_distance_vs_step.png'), 'Resolution', 150);

%% =========================================================================
%  FIG 3 — Tool shaft angle (vs world Z) over trajectory
%  =========================================================================
figure('Name','Fig3: Tool Shaft Angle','Color','w','Position',[80 80 1300 700]);
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
for s = 1:length(sc)
    nexttile; hold on; grid on;
    sa = shaft_hist(robot, results{s}.a.q, ref);
    sb = shaft_hist(robot, results{s}.b.q, ref);
    plot(sa, 'b-',  'LineWidth',1.5,'DisplayName','(a)');
    plot(sb, 'r--', 'LineWidth',1.5,'DisplayName','(b)');
    xlabel('Step'); ylabel('Shaft angle (deg)');
    title({sc(s).label, sprintf('(a) %.1f°   (b) %.1f°', max(sa)-min(sa), max(sb)-min(sb))}, ...
        'FontSize', 8);
    if s == 1, legend('Location','best'); end
    hold off;
end
sgtitle('Tool Shaft Direction — All Scenarios', 'Color', 'k');
exportgraphics(gcf, fullfile(fig_dir, 'fig3_shaft_angle.png'), 'Resolution', 150);

%% =========================================================================
%  FIG 4 — Joint angles for Cfg 3 (joint-limit pressure)
%  =========================================================================
figure('Name','Fig4: Joint Angles - Cfg3 (limit pressure)','Color','w','Position',[80 80 1100 750]);
jnames = {'J1','J2','J3','J4','J5','J6'};
lims = robot.joint_limits;
ax4_j1 = [];
for j = 1:6
    ax = subplot(2,3,j); hold on;
    if j == 1, ax4_j1 = ax; end
    plot(rad2deg(results{3}.a.q(j,:)), 'b-',  'LineWidth',1.5,'DisplayName','(a)');
    plot(rad2deg(results{3}.b.q(j,:)), 'r--', 'LineWidth',1.5,'DisplayName','(b)');
    yline(rad2deg(lims(j,1)), 'k:', 'HandleVisibility','off');
    yline(rad2deg(lims(j,2)), 'k:', 'HandleVisibility','off');
    if j > 3, xlabel('Step'); end
    ylabel('deg'); title(jnames{j}); grid on;
    hold off;
end
% Row-specific margin adjustments:
%   bottom row — shift up to clear the shared legend
%   top row    — trim top edge to clear the sgtitle
for ax = findobj(gcf, 'Type', 'axes')'
    pos = ax.Position;
    if pos(2) < 0.5   % bottom row
        ax.Position = [pos(1), pos(2) + 0.07, pos(3), pos(4) - 0.07];
    else              % top row
        ax.Position = [pos(1), pos(2),         pos(3), pos(4) - 0.04];
    end
end
% Single horizontal legend centred below all subplots (uses stored handle)
lgd4 = legend(ax4_j1, {'(a)', '(b)'}, 'Orientation','horizontal');
lgd4.Position = [0.38 0.01 0.25 0.04];
sgtitle('Cfg3 — Joint Angles vs Step (J2 hits upper limit at +35°)', 'Color', 'k');
exportgraphics(gcf, fullfile(fig_dir, 'fig4_joint_angles_cfg3.png'), 'Resolution', 150);

%% =========================================================================
%  FIG 5 — Wall deflection: Y trajectory for the two wall scenarios
%  =========================================================================
figure('Name','Fig5: Wall Y Deflection','Color','w','Position',[80 80 1100 460]);
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

nexttile;  % Wall 1 (blocking)
hold on; grid on;
pa1 = results{5}.a.p;  pb1 = results{5}.b.p;
plot(pa1(2,:), 'b-',  'LineWidth',2,'DisplayName','(c-a) wall1');
plot(pb1(2,:), 'r--', 'LineWidth',2,'DisplayName','(c-b) wall1');
yline(sc(5).p_goal(2), 'k:', 'p_{goal} Y=0.40','LabelHorizontalAlignment','right');
yline(sc(5).wall.p(2),               'Color',[0.85 0.33 0.10],'LineStyle','-', ...
    'Label','Wall Y=0.20','LabelHorizontalAlignment','right');
yline(sc(5).wall.p(2)-sc(5).wall.margin, 'Color',[0.85 0.33 0.10],'LineStyle',':', ...
    'Label','Activation Y=0.18','LabelHorizontalAlignment','right');
xlabel('Step'); ylabel('Tool tip Y (m)');
title('Cfg5 — Wall blocks goal'); legend('Location','northwest'); hold off;

nexttile;  % Wall 2 (approach limiter, goal accessible)
hold on; grid on;
pa2 = results{6}.a.p;  pb2 = results{6}.b.p;
plot(pa2(2,:), 'b-',  'LineWidth',2,'DisplayName','(c-a) wall2');
plot(pb2(2,:), 'r--', 'LineWidth',2,'DisplayName','(c-b) wall2');
yline(sc(6).p_goal(2), 'k:', 'p_{goal} Y=0.25','LabelHorizontalAlignment','right');
yline(sc(6).wall.p(2), 'Color',[0.85 0.33 0.10],'LineStyle','-', ...
    'Label','Wall Y=0.35','LabelHorizontalAlignment','right');
xlabel('Step'); ylabel('Tool tip Y (m)');
title('Cfg6 — Wall as approach limiter'); legend('Location','northwest'); hold off;

sgtitle('Virtual Wall Effect — Y-Trajectory', 'Color', 'k');
exportgraphics(gcf, fullfile(fig_dir, 'fig5_wall_deflection.png'), 'Resolution', 150);

%% =========================================================================
%  FIG 6 — Convergence comparison: which mode reaches a given tolerance first
%  =========================================================================
figure('Name','Fig6: Convergence to 1mm','Color','w','Position',[80 80 800 400]);
hold on; grid on;
labels = strings(1, length(sc));
steps_a = zeros(1, length(sc));
steps_b = zeros(1, length(sc));
for s = 1:length(sc)
    da = arrayfun(@(t) t.dist, results{s}.a.info) * 1e3;
    db = arrayfun(@(t) t.dist, results{s}.b.info) * 1e3;
    ka = find(da < 1, 1);  if isempty(ka), ka = N_MAX; end
    kb = find(db < 1, 1);  if isempty(kb), kb = N_MAX; end
    steps_a(s) = ka;  steps_b(s) = kb;
    labels(s)  = sc(s).label;
end
bar([steps_a; steps_b]', 'grouped');
set(gca,'XTickLabel',cellstr(labels));
xtickangle(30);
ylabel({'Steps to reach <1 mm', '(600 = did not converge)'});
title('Convergence Speed: (a) vs (b)', 'Color', 'k');
legend({'(a) no shaft stab.','(b) with shaft stab.'},'Location','northwest');
hold off;
exportgraphics(gcf, fullfile(fig_dir, 'fig6_convergence_speed.png'), 'Resolution', 150);

fprintf('\nAll figures generated. Saved to: %s\n', fig_dir);

if RUN_ANIMATIONS

%% =========================================================================
%  ANIM 1 — Tool shaft stabilisation: Cfg4 mode (a) vs (b)
%  Side-by-side 3D view.  Each frame: trajectory so far + current shaft.
%  =========================================================================
s_a1  = 4;
qa_a1 = results{s_a1}.a.q;   pa_a1 = results{s_a1}.a.p;
qb_a1 = results{s_a1}.b.q;   pb_a1 = results{s_a1}.b.p;
K_a1  = min(size(pa_a1,2), size(pb_a1,2));

% Fixed axis limits from both trajectories
all1  = [pa_a1, pb_a1];
xl1   = [min(all1(1,:))-0.15,  max(all1(1,:))+0.15];
yl1   = [min(all1(2,:))-0.15,  max(all1(2,:))+0.15];
zl1   = [min(all1(3,:))-0.15,  max(all1(3,:))+0.15];
goal1 = sc(s_a1).p_goal;

fig_a1 = figure('Color','w','Position',[80 80 1100 500]);
v1 = VideoWriter(fullfile(fig_dir,'anim1_shaft_cfg4.avi'),'Motion JPEG AVI');
v1.FrameRate = 15;  v1.Quality = 90;  open(v1);

skip1 = 2;
fprintf('Rendering ANIM 1 (%d frames)...\n', ceil(K_a1/skip1));
for k = 1:skip1:K_a1
    clf(fig_a1);  set(fig_a1,'Color','w');

    % --- Mode (a) left panel ---
    subplot(1,2,1);  hold on;  grid on;  view([-35 25]);
    plot3(pa_a1(1,1:k), pa_a1(2,1:k), pa_a1(3,1:k), 'b-', 'LineWidth',1.5);
    T_a  = FK_space(robot.M, robot.Slist, qa_a1(:,k));
    d_a  = T_a(1:3,1:3) * [0;0;1];
    pt_a = pa_a1(:,k);
    plot3([pt_a(1)-d_a(1)*0.2, pt_a(1)+d_a(1)*0.1], ...
          [pt_a(2)-d_a(2)*0.2, pt_a(2)+d_a(2)*0.1], ...
          [pt_a(3)-d_a(3)*0.2, pt_a(3)+d_a(3)*0.1], 'b-', 'LineWidth',4);
    plot3(goal1(1),goal1(2),goal1(3),'k*','MarkerSize',10);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    xlim(xl1); ylim(yl1); zlim(zl1);
    title(sprintf('Mode (a)  step %d',k),'Color','k');

    % --- Mode (b) right panel ---
    subplot(1,2,2);  hold on;  grid on;  view([-35 25]);
    plot3(pb_a1(1,1:k), pb_a1(2,1:k), pb_a1(3,1:k), 'r-', 'LineWidth',1.5);
    T_b  = FK_space(robot.M, robot.Slist, qb_a1(:,k));
    d_b  = T_b(1:3,1:3) * [0;0;1];
    pt_b = pb_a1(:,k);
    plot3([pt_b(1)-d_b(1)*0.2, pt_b(1)+d_b(1)*0.1], ...
          [pt_b(2)-d_b(2)*0.2, pt_b(2)+d_b(2)*0.1], ...
          [pt_b(3)-d_b(3)*0.2, pt_b(3)+d_b(3)*0.1], 'r-', 'LineWidth',4);
    plot3(goal1(1),goal1(2),goal1(3),'k*','MarkerSize',10);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    xlim(xl1); ylim(yl1); zlim(zl1);
    title(sprintf('Mode (b)  step %d',k),'Color','k');

    sgtitle('Cfg4 — Tool Shaft Stabilisation: (a) vs (b)','Color','k');
    drawnow;
    writeVideo(v1, getframe(fig_a1));
end
close(v1);
fprintf('Saved: anim1_shaft_cfg4.avi\n');

%% =========================================================================
%  ANIM 2 — Virtual wall deflection: Cfg5 both modes overlaid
%  Single 3D view with wall plane, moving tip dots.
%  =========================================================================
s_a2  = 5;
pa_a2 = results{s_a2}.a.p;
pb_a2 = results{s_a2}.b.p;
K_a2  = min(size(pa_a2,2), size(pb_a2,2));

all2  = [pa_a2, pb_a2];
xl2   = [min(all2(1,:))-0.15,  max(all2(1,:))+0.15];
yl2   = [min(all2(2,:))-0.15,  max(all2(2,:))+0.15];
zl2   = [min(all2(3,:))-0.15,  max(all2(3,:))+0.15];
goal2 = sc(s_a2).p_goal;
wp2   = sc(s_a2).wall.p;
xr2   = linspace(wp2(1)-0.5, wp2(1)+0.5, 2);
zr2   = linspace(wp2(3)-0.3, wp2(3)+0.3, 2);
[xw2, zw2] = meshgrid(xr2, zr2);

fig_a2 = figure('Color','w','Position',[80 80 900 700]);
v2 = VideoWriter(fullfile(fig_dir,'anim2_wall_cfg5.avi'),'Motion JPEG AVI');
v2.FrameRate = 15;  v2.Quality = 90;  open(v2);

skip2 = 2;
fprintf('Rendering ANIM 2 (%d frames)...\n', ceil(K_a2/skip2));
for k = 1:skip2:K_a2
    clf(fig_a2);  set(fig_a2,'Color','w');
    ax2 = axes('Parent',fig_a2);
    hold(ax2,'on');  grid(ax2,'on');  view(ax2,[-35 25]);

    % Wall plane
    surf(ax2, xw2, wp2(2)*ones(size(xw2)), zw2, ...
        'FaceAlpha',0.25,'FaceColor',[0.85 0.33 0.10],'EdgeColor','none');

    % Trajectories so far
    plot3(ax2, pa_a2(1,1:k), pa_a2(2,1:k), pa_a2(3,1:k), 'b-',  'LineWidth',2,'DisplayName','(a)');
    plot3(ax2, pb_a2(1,1:k), pb_a2(2,1:k), pb_a2(3,1:k), 'r--', 'LineWidth',2,'DisplayName','(b)');

    % Current tip positions
    plot3(ax2, pa_a2(1,k), pa_a2(2,k), pa_a2(3,k), 'bo','MarkerSize',8,'MarkerFaceColor','b','HandleVisibility','off');
    plot3(ax2, pb_a2(1,k), pb_a2(2,k), pb_a2(3,k), 'ro','MarkerSize',8,'MarkerFaceColor','r','HandleVisibility','off');

    % Goal marker
    plot3(ax2, goal2(1),goal2(2),goal2(3),'k*','MarkerSize',10,'HandleVisibility','off');

    xlabel(ax2,'X'); ylabel(ax2,'Y'); zlabel(ax2,'Z');
    xlim(ax2,xl2); ylim(ax2,yl2); zlim(ax2,zl2);
    legend(ax2,'Location','best');
    title(ax2, sprintf('Cfg5 — Virtual Wall Deflection  step %d',k),'Color','k');
    drawnow;
    writeVideo(v2, getframe(fig_a2));
end
close(v2);
fprintf('Saved: anim2_wall_cfg5.avi\n');

end  % RUN_ANIMATIONS

% Restore figure visibility so subsequent MATLAB work isn't affected
set(groot, 'defaultFigureVisible', 'on');

%% =========================================================================
%  LOCAL HELPERS
%  =========================================================================

function s = scen(label, q0, p_goal, wall, desc)
    s.label  = label;
    s.q0     = q0;
    s.p_goal = p_goal;
    s.wall   = wall;
    s.desc   = desc;
end

function angles = shaft_hist(robot, q_hist, ref)
    K = size(q_hist, 2);
    angles = zeros(1, K);
    for k = 1:K
        T = FK_space(robot.M, robot.Slist, q_hist(:,k));
        d = T(1:3,1:3) * [0;0;1];
        angles(k) = atan2d(norm(cross(d, ref)), dot(d, ref));
    end
end

function sw = shaft_swing(robot, q_hist, ref)
    a = shaft_hist(robot, q_hist, ref);
    sw = max(a) - min(a);
end

function hits = joint_limit_hits(q_hist, limits)
    hits = {};
    for j = 1:size(q_hist,1)
        if max(q_hist(j,:)) >= limits(j,2) - 1e-3
            hits{end+1} = sprintf('J%d+',j);
        end
        if min(q_hist(j,:)) <= limits(j,1) + 1e-3
            hits{end+1} = sprintf('J%d-',j);
        end
    end
end
