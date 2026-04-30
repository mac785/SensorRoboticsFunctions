%% test_functions_THA4.m — Unit tests for tool_tip_fk and QP_step_VF
%
%  T1: tool_tip_fk  — tip position matches direct FK computation
%  T2: tool_tip_fk  — Jacobian validated by finite differences (3 configs)
%  T3: QP_step_VF   — unconstrained step moves tip toward goal
%  T4: QP_step_VF   — sphere constraint: tip inside sphere cannot escape
%  T5: QP_step_VF   — wall constraint: step never penetrates wall plane
%  T6: QP_step_VF   — joint limits never violated over a full trajectory

clear;
addpath(genpath(fileparts(mfilename('fullpath')) + "/../"));

robot  = KR120_params();
L_tool = 0.1;
T      = false(1,6);   % per-test results (true = pass)

fprintf('============================================================\n');
fprintf('  THA4 Unit Tests — tool_tip_fk  &  QP_step_VF\n');
fprintf('  Robot: KR120 R2500 Pro (6-DOF), L_tool = 100 mm\n');
fprintf('============================================================\n\n');

base_opts = struct('L_tool',L_tool,'lambda',1e-4,'mu',0, ...
                   'd_max',0.003,'step',0.02,'dq_max',0.05);

%% ================================================================
%  T1 — tool_tip_fk: tip position
%  p_tip returned by the function must equal p_ee + R_ee*[0;0;L]
%  computed directly from FK_space output.
%% ================================================================
fprintf('T1: tool_tip_fk — tip position\n');

q1 = [0.3; -0.4; 0.5; 0.2; 0.4; 0.1];
[p_tip1, ~, T_ee1] = tool_tip_fk(robot, q1, L_tool);
p_tip_ref = T_ee1(1:3,4) + T_ee1(1:3,1:3)*[0;0;L_tool];
err1 = norm(p_tip1 - p_tip_ref);

fprintf('   p_tip (function): [%8.5f  %8.5f  %8.5f] m\n', p_tip1);
fprintf('   p_tip (ref FK):   [%8.5f  %8.5f  %8.5f] m\n', p_tip_ref);
fprintf('   position error:   %.2e m\n', err1);
T(1) = err1 < 1e-10;
fprintf('   --> T1: %s\n\n', pf(T(1)));

%% ================================================================
%  T2 — tool_tip_fk: Jacobian finite-difference validation
%  Tests 3 joint configurations spanning the workspace.
%  Expected: max |J_anal - J_fd| ~ 1e-9 (near machine precision).
%  Background: using skew(r) instead of skew(p_tip) gives errors
%  of ~2.7 m/rad — the same order as the arm reach.  Fixing to
%  skew(p_tip) drops the error to ~3e-9 across all configs.
%% ================================================================
fprintf('T2: tool_tip_fk — Jacobian finite-difference (3 configs)\n');

eps_fd  = 1e-7;
configs = { [0.3; -0.4;  0.5;  0.2;  0.4;  0.1], ...
            [0.0; -0.2;  0.3;  0.5; -0.3;  0.8], ...
            [1.0;  0.1; -0.4; -0.5;  0.6; -1.0] };
T2_ok = true;
for c = 1:numel(configs)
    qc = configs{c};
    [p0, J_anal] = tool_tip_fk(robot, qc, L_tool);
    n  = numel(qc);
    J_fd = zeros(3, n);
    for j = 1:n
        dqj = zeros(n,1);  dqj(j) = eps_fd;
        J_fd(:,j) = (tool_tip_fk(robot, qc+dqj, L_tool) - p0) / eps_fd;
    end
    err = max(abs(J_anal - J_fd), [], 'all');
    ok  = err < 1e-5;
    fprintf('   Config %d: max|J_anal - J_fd| = %.3e m/rad  %s\n', c, err, ck(ok));
    if ~ok, T2_ok = false; end
end
T(2) = T2_ok;
fprintf('   --> T2: %s\n\n', pf(T(2)));

%% ================================================================
%  T3 — QP_step_VF: step direction (unconstrained, far from goal)
%  The projection of J_tip*dq onto the direction toward p_goal
%  must be positive — the tip is moving closer.
%% ================================================================
fprintf('T3: QP_step_VF — unconstrained step moves tip toward goal\n');

q3     = [0; -0.4; 0.5; 0; 0.3; 0];
p3     = [1.8; 0.4; 1.0];
[dq3, info3] = QP_step_VF(robot, q3, p3, base_opts);
[p_t3, J3]   = tool_tip_fk(robot, q3, L_tool);
e3    = p3 - p_t3;
proj3 = dot(J3*dq3, e3/norm(e3));

fprintf('   Initial dist to goal: %.2f mm\n', norm(e3)*1e3);
fprintf('   ||dq||_inf:           %.5f rad\n', norm(dq3, inf));
fprintf('   Velocity toward goal: %.5f m  %s\n', proj3, ck(proj3 > 0));
T(3) = proj3 > 0;
fprintf('   --> T3: %s\n\n', pf(T(3)));

%% ================================================================
%  T4 — QP_step_VF: sphere constraint (tip cannot escape)
%  Drive the tip into the 3 mm sphere, then verify the next step
%  does not increase the distance to p_goal.
%% ================================================================
fprintf('T4: QP_step_VF — sphere constraint: tip inside sphere cannot escape\n');

q4 = [0; -0.4; 0.5; 0; 0.3; 0];
p4 = [1.8; 0.4; 1.0];
for k = 1:300
    [dq4, info4] = QP_step_VF(robot, q4, p4, base_opts);
    q4 = q4 + dq4;
    if info4.dist < 0.003, break; end
end
dist_before = norm(tool_tip_fk(robot, q4, L_tool) - p4);
inside      = dist_before < 0.003;

[dq4b, ~]    = QP_step_VF(robot, q4, p4, base_opts);
dist_after   = norm(tool_tip_fk(robot, q4+dq4b, L_tool) - p4);

fprintf('   Tip inside sphere:   dist = %.4f mm (threshold 3 mm)  %s\n', ...
    dist_before*1e3, ck(inside));
fprintf('   Dist before step:    %.4f mm\n', dist_before*1e3);
fprintf('   Dist after step:     %.4f mm  (sphere radius: 3 mm)\n', dist_after*1e3);
fprintf('   Still inside sphere: %s\n', ck(dist_after < 0.003 + 1e-6));
T(4) = inside && dist_after < 0.003 + 1e-6;
fprintf('   --> T4: %s\n\n', pf(T(4)));

%% ================================================================
%  T5 — QP_step_VF: wall constraint (no penetration)
%  Drive the tip to the wall boundary, then verify the
%  wall-normal component of J_tip*dq is non-negative.
%% ================================================================
fprintf('T5: QP_step_VF — wall constraint: step never penetrates wall\n');

wall5.n      = [0; -1; 0];
wall5.p      = [1.9; 0.20; 0.93];
wall5.margin = 0.05;
opts5        = base_opts;
opts5.wall   = wall5;
q5           = [0; -0.4; 0.5; 0; 0.3; 0];
p5           = [1.8; 0.4; 1.0];   % goal on unsafe side of wall

for k = 1:200
    [dq5, ~] = QP_step_VF(robot, q5, p5, opts5);
    q5 = q5 + dq5;
    p5tip = tool_tip_fk(robot, q5, L_tool);
    if wall5.n' * (p5tip - wall5.p) < wall5.margin, break; end
end
p5tip      = tool_tip_fk(robot, q5, L_tool);
pen_signed = wall5.n' * (p5tip - wall5.p);   % positive = safe side
near_wall  = pen_signed < wall5.margin;

[dq5b, ~] = QP_step_VF(robot, q5, p5, opts5);
[~, J5]   = tool_tip_fk(robot, q5, L_tool);
wall_vel   = wall5.n' * J5 * dq5b;           % >= 0 means moving away or along wall

fprintf('   Signed dist to wall: %.4f m  (margin %.2f m)  %s\n', ...
    pen_signed, wall5.margin, ck(near_wall));
fprintf('   Wall-normal velocity component (n''*J*dq): %.3e  %s\n', ...
    wall_vel, ck(wall_vel >= -1e-8));
T(5) = near_wall && wall_vel >= -1e-8;
fprintf('   --> T5: %s\n\n', pf(T(5)));

%% ================================================================
%  T6 — QP_step_VF: joint limits never violated
%  Run Cfg3 (J2 driven to its upper limit +35°) for 200 steps.
%  Verify q stays within [q_min, q_max] at every step.
%  J2 is expected to saturate at +35° and stay there.
%% ================================================================
fprintf('T6: QP_step_VF — joint limits never violated (Cfg3, 200 steps)\n');

q6    = [0; 0.0; 0.3; 0; 0.0; 0];
p6    = [1.5; 0.0; -1.5];
lims  = robot.joint_limits;
j2_hist  = zeros(1, 201);
j2_hist(1) = q6(2);
violated = false;

for k = 1:200
    [dq6, info6] = QP_step_VF(robot, q6, p6, base_opts);
    q6 = q6 + dq6;
    j2_hist(k+1) = q6(2);
    if any(q6 > lims(:,2) + 1e-6) || any(q6 < lims(:,1) - 1e-6)
        violated = true;  break;
    end
    if norm(dq6, inf) < 1e-6, break; end
end
j2_max = max(j2_hist(1:k+1));
fprintf('   J2 upper limit:      %+.4f deg (%.4f rad)\n', ...
    rad2deg(lims(2,2)), lims(2,2));
fprintf('   J2 peak reached:     %+.4f deg (%.4f rad)\n', ...
    rad2deg(j2_max), j2_max);
fprintf('   J2 saturated at limit: %s\n', ck(abs(j2_max - lims(2,2)) < 1e-3));
fprintf('   Any limit violated:    %s\n', ck(~violated));
T(6) = ~violated;
fprintf('   --> T6: %s\n\n', pf(T(6)));

%% ================================================================
%  SUMMARY
%% ================================================================
fprintf('============================================================\n');
fprintf('  SUMMARY\n');
fprintf('  %-8s  %s\n', 'Test', 'Result');
fprintf('  %s\n', repmat('-',1,20));
names = {'T1 — tip position','T2 — Jacobian FD','T3 — step direction', ...
         'T4 — sphere constraint','T5 — wall constraint','T6 — joint limits'};
for i = 1:6
    fprintf('  %-24s  %s\n', names{i}, pf(T(i)));
end
fprintf('  %s\n', repmat('-',1,20));
fprintf('  Passed: %d / %d\n', sum(T), numel(T));
fprintf('============================================================\n');

%% ================================================================
%  LOCAL HELPERS
%% ================================================================

function s = pf(cond)
    if cond, s = 'PASS'; else, s = 'FAIL'; end
end

function s = ck(cond)
    if cond, s = '[OK]  '; else, s = '[FAIL]'; end
end
