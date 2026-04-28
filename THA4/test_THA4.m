%% test_THA4.m — ME384R THA4 Programming Assignment
%  KUKA KR120 R2500 Pro (Quantec Nano) with 100 mm cylindrical tool.
%
%  Part (a): approach p_goal with joint limits + 3 mm sphere constraint
%  Part (b): same + penalise change in tool shaft direction
%  Part (c): parts (a) and (b) repeated with a planar virtual wall
%  Part (d): comparison plots across all parts / both configurations

clear; close all;
addpath(genpath(fileparts(mfilename('fullpath')) + "/../"));

robot = KR120_params();

%% =========================================================================
%  SHARED SIMULATION PARAMETERS
%  =========================================================================
L_tool = 0.1;    % 100 mm tool length (m)
d_max  = 0.003;  % 3 mm sphere constraint radius (m)

opts_a.L_tool  = L_tool;
opts_a.d_max   = d_max;
opts_a.lambda  = 1e-4;
opts_a.mu      = 0;        % no shaft stabilisation
opts_a.step    = 0.02;     % max Cartesian step per iter (m)
opts_a.dq_max  = 0.05;     % max joint step per iter (rad)

opts_b      = opts_a;
opts_b.mu   = 0.01;        % shaft direction stabilisation weight

N_MAX = 800;   % max steps for free-approach (a/b)
N_WALL = 300;  % max steps for wall scenarios (c)

%% =========================================================================
%  CONFIGURATION 1 — main demonstration
%    Start: arm slightly bent, tip at [2.69, 0, 0.86]
%    Goal:  [1.8, 0.4, 1.0]  — requires motion in -X, +Y, +Z
%  =========================================================================
q0_1     = [0; -0.4; 0.5; 0; 0.3; 0];
p_goal_1 = [1.8; 0.4; 1.0];

fprintf('===  Config 1  ===\n');
p0 = tool_tip_fk(robot, q0_1);
fprintf('Start tip: [%.3f, %.3f, %.3f]  |  Goal: [%.3f, %.3f, %.3f]  |  dist: %.1f mm\n\n', ...
    p0(1),p0(2),p0(3), p_goal_1(1),p_goal_1(2),p_goal_1(3), norm(p_goal_1-p0)*1e3);

[q_a1, p_a1, inf_a1] = simulate_VF(robot, q0_1, p_goal_1, N_MAX, opts_a);
[q_b1, p_b1, inf_b1] = simulate_VF(robot, q0_1, p_goal_1, N_MAX, opts_b);

%% =========================================================================
%  CONFIGURATION 2 — different goal to test generalisation
%    Goal: [2.1, -0.3, 1.2] — motion in -X, -Y, +Z
%  =========================================================================
q0_2     = [0; -0.4; 0.5; 0; 0.3; 0];
p_goal_2 = [2.1; -0.3; 1.2];

fprintf('===  Config 2  ===\n');
p0 = tool_tip_fk(robot, q0_2);
fprintf('Start tip: [%.3f, %.3f, %.3f]  |  Goal: [%.3f, %.3f, %.3f]  |  dist: %.1f mm\n\n', ...
    p0(1),p0(2),p0(3), p_goal_2(1),p_goal_2(2),p_goal_2(3), norm(p_goal_2-p0)*1e3);

[q_a2, p_a2, inf_a2] = simulate_VF(robot, q0_2, p_goal_2, N_MAX, opts_a);
[q_b2, p_b2, inf_b2] = simulate_VF(robot, q0_2, p_goal_2, N_MAX, opts_b);

%% =========================================================================
%  PART (c) — Virtual Wall
%
%  The natural approach to p_goal_1 = [1.8, 0.4, 1.0] travels from Y=0
%  monotonically to Y=0.4.  A planar virtual wall placed at Y=0.2 (safe
%  side: Y < 0.2) intercepts the path and deflects it, so the robot
%  converges to the closest accessible point (near Y=0.2) rather than
%  the goal at Y=0.4.  This clearly changes the trajectory obtained in
%  parts (a) and (b) and illustrates the dual-constraint system.
%
%  A second wall scenario places the goal on the ACCESSIBLE side (just
%  inside the safe zone) to show the wall constraining overshoot without
%  blocking the goal entirely.
%  =========================================================================

%--- Wall scenario 1: wall between robot and goal (blocks direct approach) ---
wall1.n      = [0; -1; 0];           % outward normal → safe side is Y < 0.2
wall1.p      = [1.9;  0.20; 0.93];   % point on wall surface
wall1.margin = 0.02;                  % constraint activates within 2 cm of wall

opts_ca1 = opts_a; opts_ca1.wall = wall1;
opts_cb1 = opts_b; opts_cb1.wall = wall1;

fprintf('===  Part (c) Wall 1: Y=0.20, blocks goal at Y=0.40  ===\n');
[q_ca1, p_ca1, inf_ca1] = simulate_VF(robot, q0_1, p_goal_1, N_WALL, opts_ca1);
[q_cb1, p_cb1, inf_cb1] = simulate_VF(robot, q0_1, p_goal_1, N_WALL, opts_cb1);

%--- Wall scenario 2: goal IS accessible, wall prevents overshoot past Y=0.35 ---
wall2.n      = [0; -1; 0];           % safe side: Y < 0.35
wall2.p      = [1.9;  0.35; 0.95];
wall2.margin = 0.02;

p_goal_c2    = [1.8; 0.25; 1.0];     % goal within safe zone (Y=0.25 < 0.35)

opts_ca2 = opts_a; opts_ca2.wall = wall2;
opts_cb2 = opts_b; opts_cb2.wall = wall2;

fprintf('===  Part (c) Wall 2: Y=0.35, goal accessible at Y=0.25  ===\n');
[q_ca2, p_ca2, inf_ca2] = simulate_VF(robot, q0_1, p_goal_c2, N_WALL, opts_ca2);
[q_cb2, p_cb2, inf_cb2] = simulate_VF(robot, q0_1, p_goal_c2, N_WALL, opts_cb2);

% No-wall baseline for wall scenario 2 (to compare trajectories)
[q_a2c, p_a2c, inf_a2c] = simulate_VF(robot, q0_1, p_goal_c2, N_MAX, opts_a);
[q_b2c, p_b2c, inf_b2c] = simulate_VF(robot, q0_1, p_goal_c2, N_MAX, opts_b);

%% =========================================================================
%  PART (d) — Summary Report
%  =========================================================================
fprintf('\n=== Results Summary ===\n');
fprintf('%-20s | Steps | FinalDist(mm) | JointViol\n', 'Run');
fprintf('%s\n', repmat('-',1,58));
report_sim('(a)  Config 1', p_a1,  q_a1, inf_a1, p_goal_1, robot);
report_sim('(b)  Config 1', p_b1,  q_b1, inf_b1, p_goal_1, robot);
report_sim('(a)  Config 2', p_a2,  q_a2, inf_a2, p_goal_2, robot);
report_sim('(b)  Config 2', p_b2,  q_b2, inf_b2, p_goal_2, robot);
report_sim('(c-a) Wall1',  p_ca1, q_ca1, inf_ca1, p_goal_1, robot);
report_sim('(c-b) Wall1',  p_cb1, q_cb1, inf_cb1, p_goal_1, robot);
report_sim('(c-a) Wall2',  p_ca2, q_ca2, inf_ca2, p_goal_c2, robot);
report_sim('(c-b) Wall2',  p_cb2, q_cb2, inf_cb2, p_goal_c2, robot);

% Tool shaft comparison
fprintf('\n=== Tool Shaft Direction (swing relative to world Z) ===\n');
ref = [0;0;1];
shaft_compare('(a) Config 1', robot, q_a1,  ref);
shaft_compare('(b) Config 1', robot, q_b1,  ref);
shaft_compare('(a) Config 2', robot, q_a2,  ref);
shaft_compare('(b) Config 2', robot, q_b2,  ref);

%% =========================================================================
%  PLOTS
%  =========================================================================

%--- Figure 1: 3D trajectories, Config 1, parts a/b vs c ---
figure('Name','Fig1: 3D Trajectories Config1','Color','w','Position',[100 100 900 600]);
hold on; grid on; axis equal; view([-35 25]);
plot3(p_a1(1,:),  p_a1(2,:),  p_a1(3,:),  'b-',  'LineWidth',2,'DisplayName','(a) no wall');
plot3(p_b1(1,:),  p_b1(2,:),  p_b1(3,:),  'r--', 'LineWidth',2,'DisplayName','(b) shaft stab.');
plot3(p_ca1(1,:), p_ca1(2,:), p_ca1(3,:), 'g-',  'LineWidth',2,'DisplayName','(c-a) wall 1');
plot3(p_cb1(1,:), p_cb1(2,:), p_cb1(3,:), 'm--', 'LineWidth',2,'DisplayName','(c-b) wall 1');
% Goal marker + 3 mm sphere
plot3(p_goal_1(1),p_goal_1(2),p_goal_1(3),'k*','MarkerSize',12,'HandleVisibility','off');
text(p_goal_1(1)+0.02, p_goal_1(2), p_goal_1(3)+0.02, 'p_{goal}','FontSize',10);
[sx,sy,sz] = sphere(20);
surf(d_max*sx+p_goal_1(1), d_max*sy+p_goal_1(2), d_max*sz+p_goal_1(3), ...
    'FaceAlpha',0.15,'FaceColor','k','EdgeColor','none','HandleVisibility','off');
% Start marker
p0 = tool_tip_fk(robot, q0_1);
plot3(p0(1),p0(2),p0(3),'ko','MarkerSize',8,'MarkerFaceColor','k','HandleVisibility','off');
text(p0(1)+0.02,p0(2),p0(3),'Start','FontSize',9);
% Virtual wall plane (Y = 0.2)
yw = wall1.p(2);
xr = linspace(1.5, 2.8, 2); zr = linspace(0.7, 1.3, 2);
[xw,zw] = meshgrid(xr,zr);
surf(xw, yw*ones(size(xw)), zw, 'FaceAlpha',0.2,'FaceColor',[0.3 0.8 0.8],...
    'EdgeColor','none','DisplayName','Virtual wall (Y=0.20)');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Tool Tip Trajectories — Config 1');
legend('Location','northwest'); hold off;

%--- Figure 2: Distance to goal vs step, both configs ---
figure('Name','Fig2: Distance to Goal','Color','w','Position',[100 100 950 420]);
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

nexttile;
dist_plot(inf_a1,'b-','(a)');   hold on;
dist_plot(inf_b1,'r--','(b)');
dist_plot(inf_ca1,'g-','(c-a) wall1');
dist_plot(inf_cb1,'m--','(c-b) wall1');
yline(d_max*1e3,'k:','3 mm','LabelHorizontalAlignment','left');
xlabel('Step'); ylabel('Dist to goal (mm)'); title('Config 1 — Goal [1.8, 0.4, 1.0]');
legend('Location','northeast'); grid on; hold off; set(gca,'YScale','log');

nexttile;
dist_plot(inf_a2,'b-','(a)'); hold on;
dist_plot(inf_b2,'r--','(b)');
yline(d_max*1e3,'k:','3 mm','LabelHorizontalAlignment','left');
xlabel('Step'); ylabel('Dist to goal (mm)'); title('Config 2 — Goal [2.1, -0.3, 1.2]');
legend('Location','northeast'); grid on; hold off; set(gca,'YScale','log');

%--- Figure 3: Joint angles vs step, Config 1 (a vs b vs c-a vs c-b) ---
figure('Name','Fig3: Joint Angles','Color','w','Position',[100 100 1000 600]);
jnames = {'J1','J2','J3','J4','J5','J6'};
clrs = lines(4);
lims = robot.joint_limits;
for j = 1:6
    subplot(2,3,j); hold on;
    plot(rad2deg(q_a1(j,:)),  '-',  'Color',clrs(1,:),'DisplayName','(a)');
    plot(rad2deg(q_b1(j,:)),  '--', 'Color',clrs(2,:),'DisplayName','(b)');
    plot(rad2deg(q_ca1(j,:)), '-',  'Color',clrs(3,:),'DisplayName','(c-a)');
    plot(rad2deg(q_cb1(j,:)), '--', 'Color',clrs(4,:),'DisplayName','(c-b)');
    yline(rad2deg(lims(j,1)),'k:'); yline(rad2deg(lims(j,2)),'k:');
    xlabel('Step'); ylabel('deg'); title(jnames{j}); grid on;
    if j==1, legend('Location','best'); end
    hold off;
end
sgtitle('Joint Angles — Config 1 (dotted = limits)');

%--- Figure 4: Tool shaft direction angle vs step ---
figure('Name','Fig4: Tool Shaft','Color','w','Position',[100 100 950 420]);
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

nexttile;
sa_a1  = shaft_hist(robot, q_a1,  ref);
sa_b1  = shaft_hist(robot, q_b1,  ref);
sa_ca1 = shaft_hist(robot, q_ca1, ref);
sa_cb1 = shaft_hist(robot, q_cb1, ref);
plot(sa_a1, 'b-','LineWidth',1.5,'DisplayName','(a)'); hold on;
plot(sa_b1, 'r--','LineWidth',1.5,'DisplayName','(b) shaft stab.');
plot(sa_ca1,'g-','LineWidth',1.5,'DisplayName','(c-a) wall');
plot(sa_cb1,'m--','LineWidth',1.5,'DisplayName','(c-b) wall+shaft');
xlabel('Step'); ylabel('Angle from world Z (deg)');
title('Config 1 — Tool Shaft Direction');
legend; grid on; hold off;

nexttile;
sa_a2 = shaft_hist(robot, q_a2, ref);
sa_b2 = shaft_hist(robot, q_b2, ref);
plot(sa_a2,'b-','LineWidth',1.5,'DisplayName','(a)'); hold on;
plot(sa_b2,'r--','LineWidth',1.5,'DisplayName','(b) shaft stab.');
xlabel('Step'); ylabel('Angle from world Z (deg)');
title('Config 2 — Tool Shaft Direction');
legend; grid on; hold off;

%--- Figure 5: Y-trajectory — wall deflection effect (Wall 1) ---
figure('Name','Fig5: Wall1 Y-Deflection','Color','w','Position',[100 100 700 420]);
hold on; grid on;
plot(p_a1(2,:),  'b-',  'LineWidth',2,'DisplayName','(a) no wall');
plot(p_b1(2,:),  'r--', 'LineWidth',2,'DisplayName','(b) no wall + shaft');
plot(p_ca1(2,:), 'g-',  'LineWidth',2,'DisplayName','(c-a) + wall');
plot(p_cb1(2,:), 'm--', 'LineWidth',2,'DisplayName','(c-b) + wall + shaft');
yline(p_goal_1(2), 'k:', 'p_{goal} Y=0.40','LabelHorizontalAlignment','right','LineWidth',1.5);
yline(wall1.p(2),  'c-', 'Wall Y=0.20',    'LabelHorizontalAlignment','right','LineWidth',1.5);
yline(wall1.p(2)-wall1.margin,'c:','Activation Y=0.18','LabelHorizontalAlignment','left');
xlabel('Step'); ylabel('Tool tip Y (m)');
title('Virtual Wall Effect — Y Trajectory (Wall 1: Y=0.20)');
legend('Location','southeast'); hold off;

%--- Figure 6: Wall 2 — accessible goal near wall, 3D + Y trajectory ---
figure('Name','Fig6: Wall2 Accessible Goal','Color','w','Position',[100 100 1050 460]);
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

nexttile;  % 3D
hold on; grid on; axis equal; view([-35 25]);
plot3(p_a2c(1,:), p_a2c(2,:), p_a2c(3,:), 'b-', 'LineWidth',2,'DisplayName','(a) no wall');
plot3(p_b2c(1,:), p_b2c(2,:), p_b2c(3,:), 'r--','LineWidth',2,'DisplayName','(b) no wall');
plot3(p_ca2(1,:), p_ca2(2,:), p_ca2(3,:), 'g-', 'LineWidth',2,'DisplayName','(c-a) + wall');
plot3(p_cb2(1,:), p_cb2(2,:), p_cb2(3,:), 'm--','LineWidth',2,'DisplayName','(c-b) + wall');
plot3(p_goal_c2(1),p_goal_c2(2),p_goal_c2(3),'k*','MarkerSize',12,'HandleVisibility','off');
text(p_goal_c2(1)+0.02,p_goal_c2(2),p_goal_c2(3),'p_{goal}','FontSize',10);
yw2 = wall2.p(2);
surf(xw, yw2*ones(size(xw)), zw, 'FaceAlpha',0.2,'FaceColor',[0.3 0.8 0.8],...
    'EdgeColor','none','DisplayName','Wall Y=0.35');
p0 = tool_tip_fk(robot, q0_1);
plot3(p0(1),p0(2),p0(3),'ko','MarkerSize',8,'MarkerFaceColor','k','HandleVisibility','off');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Trajectories — Wall 2, Goal Accessible'); legend('Location','northwest'); hold off;

nexttile;  % Y vs step
hold on; grid on;
plot(p_a2c(2,:), 'b-', 'LineWidth',2,'DisplayName','(a) no wall');
plot(p_b2c(2,:), 'r--','LineWidth',2,'DisplayName','(b) no wall');
plot(p_ca2(2,:), 'g-', 'LineWidth',2,'DisplayName','(c-a) + wall');
plot(p_cb2(2,:), 'm--','LineWidth',2,'DisplayName','(c-b) + wall');
yline(p_goal_c2(2),'k:','Goal Y=0.25','LabelHorizontalAlignment','right');
yline(wall2.p(2),  'c-','Wall Y=0.35', 'LabelHorizontalAlignment','right');
xlabel('Step'); ylabel('Tool tip Y (m)');
title('Y-Trajectory — Wall 2 (Goal Accessible)');
legend('Location','southeast'); hold off;

fprintf('\nAll figures generated.\n');

%% =========================================================================
%  LOCAL HELPERS
%  =========================================================================

function report_sim(label, p_hist, q_hist, info_hist, ~, robot)
    dist_mm = arrayfun(@(s) s.dist, info_hist) * 1e3;
    viol = any(any(q_hist < robot.joint_limits(:,1) - 1e-5 | ...
                   q_hist > robot.joint_limits(:,2) + 1e-5));
    fprintf('%-20s | %5d | %13.4f | %s\n', label, size(p_hist,2)-1, ...
        dist_mm(end), ternary(viol,'YES','no'));
end

function dist_plot(info_hist, style, lbl)
    d = arrayfun(@(s) s.dist, info_hist) * 1e3;
    plot(d, style, 'LineWidth',1.5,'DisplayName',lbl);
end

function angles = shaft_hist(robot, q_hist, ref)
    K = size(q_hist,2);
    angles = zeros(1,K);
    for k = 1:K
        T = FK_space(robot.M, robot.Slist, q_hist(:,k));
        d = T(1:3,1:3)*[0;0;1];
        angles(k) = atan2d(norm(cross(d,ref)), dot(d,ref));
    end
end

function shaft_compare(label, robot, q_hist, ref)
    a = shaft_hist(robot, q_hist, ref);
    fprintf('  %-20s | swing=%.2f deg | max step=%.3f deg\n', ...
        label, max(a)-min(a), max(abs(diff(a))));
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
