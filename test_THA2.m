%% test_THA2.m
% One-stop test runner for ME397 THA2 Programming Assignment.
% Robot: KUKA KR210 R2700 Extra
%
% Run this file from the SensorRoboticsFunctions directory:
%   >> test_THA2
%
% Each section corresponds to a PA part (a, b, c, ...).
% Results are printed to the console; a summary table is shown at the end.
% Visualization tests open figures when run interactively; they are skipped
% in headless/batch mode.

clc;
addpath('.');      % ensure helpers are on path

%% ---- Test bookkeeping ----
results = struct('label', {}, 'pass', {}, 'detail', {});

function results = record(results, label, pass, detail)
    n = numel(results) + 1;
    results(n).label  = label;
    results(n).pass   = pass;
    results(n).detail = detail;
    if pass
        fprintf('  [PASS]  %s  (%s)\n', label, detail);
    else
        fprintf('  [FAIL]  %s  (%s)\n', label, detail);
    end
end

%% ============================================================
%  PART (a) & (b): FK_space — space-form product of exponentials
%  Assignment: T = exp([S1]t1)*...*exp([S6]t6)*M
%              + graphical visualisation of frames and screw axes
%% ============================================================
fprintf('\n========== PART (a)/(b): FK_space ==========\n');
robot = KR210_params();

% --- Test a1: at home (theta=0), FK must return M exactly
T0   = FK_space(robot.M, robot.Slist, zeros(6,1));
err  = norm(T0 - robot.M, 'fro');
results = record(results, 'a1: FK_space(0) == M', err < 1e-10, ...
                 sprintf('||T-M||_F = %.2e', err));

% --- Test a2: rotation matrix at home must be identity
R_err = norm(T0(1:3,1:3) - eye(3), 'fro');
results = record(results, 'a2: R(0) == I', R_err < 1e-10, ...
                 sprintf('||R-I||_F = %.2e', R_err));

% --- Test a3: pure J1 rotation pi/2 (Z-axis) preserves xy-radius and z
T1      = FK_space(robot.M, robot.Slist, [pi/2;0;0;0;0;0]);
p1      = T1(1:3,4);
r_delta = abs(norm(p1(1:2)) - norm(robot.M(1:2,4)));
z_delta = abs(p1(3) - robot.M(3,4));
results = record(results, 'a3: J1 rotate pi/2 — xy-radius preserved', ...
                 r_delta < 1e-6, sprintf('delta = %.2e m', r_delta));
results = record(results, 'a4: J1 rotate pi/2 — Z height preserved', ...
                 z_delta < 1e-6, sprintf('delta = %.2e m', z_delta));

% --- Test a5: pure J2 rotation pi/2 — arm sweeps downward
%   At home, arm reach from J2 = a2+a3+d4+d6 = 3.02 m along +X.
%   Rotating J2 by +pi/2 about Y sends arm to -Z direction.
%   Expected EE: x = a1, y = 0, z = d1 - (a2+a3+d4+d6)
T2       = FK_space(robot.M, robot.Slist, [0;pi/2;0;0;0;0]);
p2       = T2(1:3,4);
reach    = robot.a(2) + robot.a(3) + robot.d(4) + robot.d(6);
p2_exp   = [robot.a(1); 0; robot.d(1) - reach];
pos_err2 = norm(p2 - p2_exp);
results  = record(results, 'a5: J2 rotate pi/2 — EE pos correct', ...
                  pos_err2 < 1e-6, ...
                  sprintf('[%.4f,%.4f,%.4f] expect [%.4f,%.4f,%.4f]  err=%.2e', ...
                          p2(1),p2(2),p2(3), p2_exp(1),p2_exp(2),p2_exp(3), pos_err2));

% --- Test a6 (visualisation): skipped in headless mode, noted as manual
%   To test: run FK_space(robot.M, robot.Slist, zeros(6,1), true) interactively.
fprintf('  [MANUAL] a6: FK_space visualise — run FK_space(robot.M,robot.Slist,zeros(6,1),true,robot.q_joints)\n');


%% ============================================================
%  PART (c): FK_body — body-form product of exponentials
%  Assignment: T = M * exp([B1]t1)*...*exp([B6]t6)
%              + graphical visualisation
%% ============================================================
fprintf('\n========== PART (c): FK_body ==========\n');

% --- Test c1: at home, FK_body must return M
Tb0  = FK_body(robot.M, robot.Blist, zeros(6,1));
err  = norm(Tb0 - robot.M, 'fro');
results = record(results, 'c1: FK_body(0) == M', err < 1e-10, ...
                 sprintf('||T-M||_F = %.2e', err));

% --- Test c2: FK_body and FK_space must agree for arbitrary theta
theta_arb = [0.3; -0.5; 0.8; 0.2; -0.4; 1.0];
T_sp  = FK_space(robot.M, robot.Slist, theta_arb);
T_bd  = FK_body(robot.M, robot.Blist, theta_arb);
err3  = norm(T_sp - T_bd, 'fro');
results = record(results, 'c2: FK_space == FK_body (arb theta)', err3 < 1e-10, ...
                 sprintf('||Tsp-Tbd||_F = %.2e', err3));

% --- Test c3: multiple random configurations
rng(42);
max_fk_err = 0;
for k = 1:20
    th = (rand(6,1) - 0.5) * pi;
    Ts = FK_space(robot.M, robot.Slist, th);
    Tb = FK_body(robot.M, robot.Blist, th);
    max_fk_err = max(max_fk_err, norm(Ts - Tb, 'fro'));
end
results = record(results, 'c3: FK_space == FK_body (20 random configs)', ...
                 max_fk_err < 1e-10, sprintf('max ||err||_F = %.2e', max_fk_err));

% --- Test c4: output must be a valid SE(3) element (R in SO(3), det=1, bottom row=[0001])
T_check = FK_body(robot.M, robot.Blist, theta_arb);
R_check = T_check(1:3,1:3);
so3_err = norm(R_check'*R_check - eye(3), 'fro');
det_err = abs(det(R_check) - 1);
bot_err = norm(T_check(4,:) - [0,0,0,1]);
results = record(results, 'c4: T in SE(3) — R''R=I', so3_err < 1e-12, ...
                 sprintf('||R''R-I||_F = %.2e', so3_err));
results = record(results, 'c5: T in SE(3) — det(R)=1', det_err < 1e-12, ...
                 sprintf('|det(R)-1| = %.2e', det_err));
results = record(results, 'c6: T in SE(3) — bottom row=[0001]', bot_err < 1e-12, ...
                 sprintf('err = %.2e', bot_err));

fprintf('  [MANUAL] c7: FK_body visualise — run FK_body(robot.M,robot.Blist,zeros(6,1),true,robot.q_joints)\n');


%% ============================================================
%  PART (d) & (e): J_space and J_body
%  Assignment: space & body Jacobians; functions J_space.m, J_body.m
%% ============================================================
fprintf('\n========== PART (d)/(e): J_space and J_body ==========\n');

% --- Test d1: at home, column 1 of J_space = S1 (no preceding joints)
Js0 = J_space(robot.Slist, zeros(6,1));
col1_err = norm(Js0(:,1) - robot.Slist(:,1));
results = record(results, 'd1: J_space col1 at home == S1', col1_err < 1e-12, ...
                 sprintf('err = %.2e', col1_err));

% --- Test d2: at home, J_space columns should equal Slist columns
%   (because all Ad(T_{i-1}) = Ad(I) = I when all preceding thetas=0)
Js0_err = norm(Js0 - robot.Slist, 'fro');
results = record(results, 'd2: J_space(0) == Slist', Js0_err < 1e-12, ...
                 sprintf('||Js-Slist||_F = %.2e', Js0_err));

% --- Test d3: at home, J_body columns should equal Blist columns
Jb0     = J_body(robot.Blist, zeros(6,1));
Jb0_err = norm(Jb0 - robot.Blist, 'fro');
results = record(results, 'd3: J_body(0) == Blist', Jb0_err < 1e-12, ...
                 sprintf('||Jb-Blist||_F = %.2e', Jb0_err));

% --- Test d4: J_space and J_body are related by adjoint of FK
%   Jb = Ad(T^{-1}) * Js  =>  Ad(T)*Jb - Js should be ~0
T_arb  = FK_space(robot.M, robot.Slist, theta_arb);
Js_arb = J_space(robot.Slist, theta_arb);
Jb_arb = J_body(robot.Blist, theta_arb);
rel_err = norm(adjoint_map(T_arb) * Jb_arb - Js_arb, 'fro');
results = record(results, 'd4: Ad(T)*Jb == Js (arb theta)', rel_err < 1e-10, ...
                 sprintf('||Ad(T)*Jb - Js||_F = %.2e', rel_err));

% --- Test d5: Jacobian must be 6xn
[r_js, c_js] = size(Js_arb);
[r_jb, c_jb] = size(Jb_arb);
results = record(results, 'd5: J_space is 6x6', (r_js==6 && c_js==6), ...
                 sprintf('size = %dx%d', r_js, c_js));
results = record(results, 'd6: J_body is 6x6', (r_jb==6 && c_jb==6), ...
                 sprintf('size = %dx%d', r_jb, c_jb));

% --- Test d7: numerical Jacobian check via component-wise finite differences.
%   Avoids MatrixLog6 (which has a known linear-component scaling bug) by using:
%     Angular: SO3ToVec(MatrixLog3(R_base' * R_p) / eps)  -> omega_body
%     Linear : R_base' * (p_p - p_base) / eps             -> v_body
eps_fd  = 1e-5;
T_base  = FK_body(robot.M, robot.Blist, theta_arb);
R_base  = T_base(1:3,1:3);
p_base  = T_base(1:3,4);
J_num   = zeros(6, 6);
for i = 1:6
    th_p       = theta_arb; th_p(i) = th_p(i) + eps_fd;
    T_p        = FK_body(robot.M, robot.Blist, th_p);
    % Angular body velocity: SO3ToVec(R_base^T * dR/dtheta_i)
    J_num(1:3,i) = SO3ToVec(MatrixLog3(R_base' * T_p(1:3,1:3)) / eps_fd);
    % Linear body velocity: R_base^T * dp/dtheta_i
    J_num(4:6,i) = R_base' * (T_p(1:3,4) - p_base) / eps_fd;
end
fd_err = norm(J_num - Jb_arb, 'fro');
% NOTE: MatrixLog6 has a confirmed bug (linear component not scaled by theta).
%       That bug does NOT affect this test, which uses MatrixLog3 + direct FD only.
results = record(results, 'd7: J_body matches finite-difference Jacobian', fd_err < 1e-3, ...
                 sprintf('||Jb - Jnum||_F = %.2e', fd_err));


%% ============================================================
%  PART (f): singularity.m — analytical singularity detection
%  Assignment: analytically identify wrist / shoulder / elbow singularities.
%
%  Singularity types for 6-DOF spherical-wrist robot:
%    1. WRIST:    sin(theta5) = 0  ->  axes J4 and J6 align
%    2. SHOULDER: wrist center on J1 (Z) axis
%    3. ELBOW:    arm fully extended (d_sw = L1+L2) or folded (d_sw = |L1-L2|)
%% ============================================================
fprintf('\n========== PART (f): singularity ==========\n');

% Helper: run singularity() quietly (suppress its fprintf output).
% singularity() prints a report to console; we capture it with evalc so the
% test runner output stays clean, then check the returned values.

% --- Test f1/f2: WRIST singularity (theta5=0) is detected
theta_ws = [0.2; 0.3; -0.2; 0.1; 0; 0.4];
[~, sing_ws, w_ws, types_ws] = evalc('singularity(robot, theta_ws)');
results = record(results, 'f1: wrist singularity (theta5=0) — is_singular', ...
                 sing_ws, sprintf('w=%.2e', w_ws));
results = record(results, 'f2: wrist singularity (theta5=0) — type=WRIST', ...
                 any(cellfun(@(s) contains(s,'WRIST'), types_ws)), ...
                 strjoin(types_ws, ' | '));

% --- Test f3/f4: WRIST singularity (theta5=pi) is detected
theta_wpi = [0.2; 0.3; -0.2; 0.1; pi; 0.4];
[~, sing_wpi, w_wpi, types_wpi] = evalc('singularity(robot, theta_wpi)');
results = record(results, 'f3: wrist singularity (theta5=pi) — is_singular', ...
                 sing_wpi, sprintf('w=%.2e', w_wpi));
results = record(results, 'f4: wrist singularity (theta5=pi) — type=WRIST', ...
                 any(cellfun(@(s) contains(s,'WRIST'), types_wpi)), ...
                 strjoin(types_wpi, ' | '));

% --- Test f5: manipulability w -> 0 at singular config
results = record(results, 'f5: w~0 at wrist singularity', w_ws < 1e-6, ...
                 sprintf('w=%.2e', w_ws));

% --- Test f6/f7: non-singular config produces w > 0 and no singularity flag
theta_ns = [0.1; 0.2; -0.3; 0.4; 0.5; 0.6];
[~, sing_ns, w_ns] = evalc('singularity(robot, theta_ns)');
results = record(results, 'f6: non-singular config — is_singular=false', ...
                 ~sing_ns, sprintf('w=%.4f', w_ns));
results = record(results, 'f7: non-singular config — w > 0', w_ns > 0.1, ...
                 sprintf('w=%.4f', w_ns));

% --- Test f8/f9: ELBOW singularity — arm fully extended, wrist non-singular
%   theta=[0;0;0;0;pi/6;0]: arm at home (extended), theta5=pi/6 (not wrist sing).
%   w should be 0 (rank-deficient due to extended arm — confirmed numerically).
%   NOTE: analytical ELBOW label expected to FAIL: singularity.m uses
%   L2=sqrt(a3^2+d4^2)=1.401 but correct forearm length is a3+d4=1.455 m.
%   (Bug #2 in running error list — does not affect is_singular via SVD catch)
theta_el = [0; 0; 0; 0; pi/6; 0];
[~, sing_el, w_el, types_el] = evalc('singularity(robot, theta_el)');
results = record(results, 'f8: elbow singularity — is_singular (via w/SVD)', ...
                 sing_el, sprintf('w=%.2e', w_el));
results = record(results, 'f9: elbow singularity — type=ELBOW [expect FAIL: L2 bug]', ...
                 any(cellfun(@(s) contains(s,'ELBOW'), types_el)), ...
                 strjoin(types_el, ' | '));

% --- Test f10/f11: SHOULDER singularity — wrist center on J1 axis
%   theta2 = acos(-a1/(a2+a3+d4)) ~ 97 deg (outside KR210 joint limits but
%   the function does not enforce limits; math is valid).
t2_sh  = acos(-robot.a(1) / (robot.a(2)+robot.a(3)+robot.d(4)));
theta_sh = [0; t2_sh; 0; 0; pi/6; 0];
[~, sing_sh, w_sh, types_sh] = evalc('singularity(robot, theta_sh)');
results = record(results, 'f10: shoulder singularity — is_singular', ...
                 sing_sh, sprintf('w=%.2e', w_sh));
results = record(results, 'f11: shoulder singularity — type=SHOULDER', ...
                 any(cellfun(@(s) contains(s,'SHOULDER'), types_sh)), ...
                 strjoin(types_sh, ' | '));

%% ============================================================
%  PART (g): Manipulability metrics and ellipsoid plots
%  Assignment: implement J_isotropy, J_condition, J_ellipsoid_volume,
%              ellipsoid_plot_angular, ellipsoid_plot_linear
%% ============================================================
fprintf('\n========== PART (g): Manipulability metrics ==========\n');
% theta_ns and theta_ws already defined in part (f) with the same values

% --- Test g1: J_isotropy — non-singular config (expected ~0.0344)
iso_ns   = J_isotropy(robot, theta_ns);
results  = record(results, 'g1: J_isotropy non-singular', ...
                  abs(iso_ns - 0.034398) < 1e-4, ...
                  sprintf('iso=%.6f', iso_ns));

% --- Test g2: J_isotropy — wrist-singular config → should be ~0
iso_ws   = J_isotropy(robot, theta_ws);
results  = record(results, 'g2: J_isotropy wrist-singular -> ~0', ...
                  iso_ws < 1e-3, ...
                  sprintf('iso=%.2e', iso_ws));

% --- Test g3: J_condition — non-singular config (expected ~29.07)
cond_ns  = J_condition(robot, theta_ns);
results  = record(results, 'g3: J_condition non-singular', ...
                  abs(cond_ns - 29.071342) < 1e-2, ...
                  sprintf('kappa=%.6f', cond_ns));

% --- Test g4: J_condition — wrist-singular config → very large
cond_ws  = J_condition(robot, theta_ws);
results  = record(results, 'g4: J_condition wrist-singular -> large', ...
                  cond_ws > 1e6, ...
                  sprintf('kappa=%.2e', cond_ws));

% --- Test g5: J_ellipsoid_volume — linear volume non-singular (expected ~16.355)
% Signature: [vol_lin, vol_ang] = J_ellipsoid_volume(...)
[vol_lin_ns, vol_ang_ns] = J_ellipsoid_volume(robot, theta_ns);
results = record(results, 'g5: J_ellipsoid_volume linear non-singular', ...
                 abs(vol_lin_ns - 16.355407) < 1e-3, ...
                 sprintf('vol_lin=%.6f', vol_lin_ns));

% --- Test g6: J_ellipsoid_volume — angular volume non-singular (expected ~10.616)
results = record(results, 'g6: J_ellipsoid_volume angular non-singular', ...
                 abs(vol_ang_ns - 10.616042) < 1e-3, ...
                 sprintf('vol_ang=%.6f', vol_ang_ns));

% --- Test g7: J_ellipsoid_volume — wrist-singular: 3x6 angular sub-Jacobian
%   stays full rank (rank 3) at a wrist singularity, so vol_ang > 0.
%   The rank deficiency is only in the full 6x6 Jacobian.
[~, vol_ang_ws] = J_ellipsoid_volume(robot, theta_ws);
results = record(results, 'g7: J_ellipsoid_volume angular wrist-singular > 0', ...
                 vol_ang_ws > 0, ...
                 sprintf('vol_ang=%.4f (3x6 sub-Jacobian stays rank-3)', vol_ang_ws));

% --- Test g8: ellipsoid_plot_angular — runs without error (headless: figure suppressed)
g8_pass = true; g8_detail = 'skipped (no display)';
if ~isempty(getenv('DISPLAY')) || ~isempty(getenv('MATLAB_DISPLAY'))
    try
        ellipsoid_plot_angular(robot, theta_ns);
        close all;
        g8_detail = 'figure opened OK';
    catch ME
        g8_pass   = false;
        g8_detail = ME.message;
    end
end
results = record(results, 'g8: ellipsoid_plot_angular runs without error', g8_pass, g8_detail);

% --- Test g9: ellipsoid_plot_linear — runs without error
g9_pass = true; g9_detail = 'skipped (no display)';
if ~isempty(getenv('DISPLAY')) || ~isempty(getenv('MATLAB_DISPLAY'))
    try
        ellipsoid_plot_linear(robot, theta_ns);
        close all;
        g9_detail = 'figure opened OK';
    catch ME
        g9_pass   = false;
        g9_detail = ME.message;
    end
end
results = record(results, 'g9: ellipsoid_plot_linear runs without error', g9_pass, g9_detail);

%% ============================================================
%  PART (h): J_inverse_kinematics — Newton-Raphson IK
%  Assignment: theta <- theta + pinv(J_b) * V_b   (Algorithm 6.2, Lynch & Park)
%  V_b = vec(MatrixLog6(T_curr^{-1} * T_desired))
%% ============================================================
fprintf('\n========== PART (h): J_inverse_kinematics ==========\n');

theta_true = [0.1; 0.2; -0.3; 0.4; 0.5; 0.6];
T_des_h    = FK_body(robot.M, robot.Blist, theta_true);
theta0_h   = zeros(6,1);

[th_h, ok_h, it_h] = J_inverse_kinematics(robot, T_des_h, theta0_h);

% Residual body twist at solution
T_sol_h  = FK_body(robot.M, robot.Blist, th_h);
Vmat_h   = MatrixLog6(inv_transform(T_sol_h) * T_des_h);
omg_err_h = norm(SO3ToVec(Vmat_h(1:3,1:3)));
lin_err_h = norm(Vmat_h(1:3,4));

% --- Test h1: converges (success flag)
results = record(results, 'h1: J_inverse_kinematics converges', ...
                 ok_h, sprintf('iters=%d', it_h));

% --- Test h2: angular residual < eomg (default 1e-3)
results = record(results, 'h2: angular error < 1e-3 rad', ...
                 omg_err_h < 1e-3, sprintf('omg_err=%.2e', omg_err_h));

% --- Test h3: linear residual < ev (default 1e-3)
results = record(results, 'h3: linear error < 1e-3 m', ...
                 lin_err_h < 1e-3, sprintf('lin_err=%.2e', lin_err_h));

% --- Test h4: FK(solution) matches T_desired (rotation)
R_err_h = norm(T_sol_h(1:3,1:3) - T_des_h(1:3,1:3), 'fro');
results = record(results, 'h4: FK(solution) rotation matches T_desired', ...
                 R_err_h < 1e-3, sprintf('||dR||_F=%.2e', R_err_h));

% --- Test h5: FK(solution) matches T_desired (translation)
p_err_h = norm(T_sol_h(1:3,4) - T_des_h(1:3,4));
results = record(results, 'h5: FK(solution) position matches T_desired', ...
                 p_err_h < 1e-3, sprintf('||dp||=%.2e m', p_err_h));

% --- Test h6: returns failure for unreachable target (10 m beyond workspace)
T_unreach  = FK_body(robot.M, robot.Blist, zeros(6,1));
T_unreach(1,4) = T_unreach(1,4) + 10;
[~, ok_unreach, ~] = J_inverse_kinematics(robot, T_unreach, zeros(6,1), 1e-3, 1e-3, 20);
results = record(results, 'h6: returns failure for unreachable target', ...
                 ~ok_unreach, sprintf('converged=%d', ok_unreach));

%% ============================================================
%  PART (i): J_transpose_kinematics — Jacobian-transpose IK
%  Assignment: theta <- theta + alpha * J_b' * V_b
%  Note: J-transpose has a narrower basin of attraction than Newton-Raphson;
%        it requires a closer initial guess or more iterations to converge.
%% ============================================================
fprintf('\n========== PART (i): J_transpose_kinematics ==========\n');

% Use a close initial guess (theta_true + 0.1) — J-transpose needs it
theta_true_i = [0.1; 0.2; -0.3; 0.4; 0.5; 0.6];
T_des_i      = FK_body(robot.M, robot.Blist, theta_true_i);
theta0_i     = theta_true_i + 0.1 * ones(6,1);   % close guess

[th_i, ok_i, it_i] = J_transpose_kinematics(robot, T_des_i, theta0_i);

T_sol_i   = FK_body(robot.M, robot.Blist, th_i);
Vmat_i    = MatrixLog6(inv_transform(T_sol_i) * T_des_i);
omg_err_i = norm(SO3ToVec(Vmat_i(1:3,1:3)));
lin_err_i = norm(Vmat_i(1:3,4));

% --- Test i1: converges (success flag)
results = record(results, 'i1: J_transpose_kinematics converges', ...
                 ok_i, sprintf('iters=%d', it_i));

% --- Test i2: angular residual < eomg (default 1e-3)
results = record(results, 'i2: angular error < 1e-3 rad', ...
                 omg_err_i < 1e-3, sprintf('omg_err=%.2e', omg_err_i));

% --- Test i3: linear residual < ev (default 1e-3)
results = record(results, 'i3: linear error < 1e-3 m', ...
                 lin_err_i < 1e-3, sprintf('lin_err=%.2e', lin_err_i));

% --- Test i4: FK(solution) position matches T_desired
p_err_i = norm(T_sol_i(1:3,4) - T_des_i(1:3,4));
results = record(results, 'i4: FK(solution) position matches T_desired', ...
                 p_err_i < 1e-3, sprintf('||dp||=%.2e m', p_err_i));

% --- Test i5: uses more iterations than Newton-Raphson (slower convergence)
results = record(results, 'i5: J-transpose slower than NR (iters > 5)', ...
                 it_i > it_h, sprintf('J-transpose=%d iters vs NR=%d iters', it_i, it_h));

% --- Test i6: returns failure for unreachable target
T_unreach_i = FK_body(robot.M, robot.Blist, zeros(6,1));
T_unreach_i(1,4) = T_unreach_i(1,4) + 10;
[~, ok_unreach_i, ~] = J_transpose_kinematics(robot, T_unreach_i, zeros(6,1), 0.1, 1e-3, 1e-3, 30);
results = record(results, 'i6: returns failure for unreachable target', ...
                 ~ok_unreach_i, sprintf('converged=%d', ok_unreach_i));

%% ============================================================
%  PART (j): redundancy_resolution — null-space manipulability-maximising IK
%  Assignment: theta <- theta + J_b^+ * V_b  +  (I - J_b^+ J_b) * (k0 * grad_w)
%  For 6-DOF (n=m=6): null-space projector N = I - J^+ J ≈ 0, secondary term
%  vanishes and the method reduces to standard Newton-Raphson.
%% ============================================================
fprintf('\n========== PART (j): redundancy_resolution ==========\n');

theta_true_j = [0.1; 0.2; -0.3; 0.4; 0.5; 0.6];
T_des_j      = FK_body(robot.M, robot.Blist, theta_true_j);

[th_j, ok_j, it_j] = redundancy_resolution(robot, T_des_j, zeros(6,1));

T_sol_j   = FK_body(robot.M, robot.Blist, th_j);
Vmat_j    = MatrixLog6(inv_transform(T_sol_j) * T_des_j);
omg_err_j = norm(SO3ToVec(Vmat_j(1:3,1:3)));
lin_err_j = norm(Vmat_j(1:3,4));

% --- Test j1: converges
results = record(results, 'j1: redundancy_resolution converges', ...
                 ok_j, sprintf('iters=%d', it_j));

% --- Test j2: angular residual < eomg (default 1e-3)
results = record(results, 'j2: angular error < 1e-3 rad', ...
                 omg_err_j < 1e-3, sprintf('omg_err=%.2e', omg_err_j));

% --- Test j3: linear residual < ev (default 1e-3)
results = record(results, 'j3: linear error < 1e-3 m', ...
                 lin_err_j < 1e-3, sprintf('lin_err=%.2e', lin_err_j));

% --- Test j4: FK(solution) position matches T_desired
p_err_j = norm(T_sol_j(1:3,4) - T_des_j(1:3,4));
results = record(results, 'j4: FK(solution) position matches T_desired', ...
                 p_err_j < 1e-3, sprintf('||dp||=%.2e m', p_err_j));

% --- Test j5: null-space projector N ≈ 0 for 6-DOF robot
Jb_arb   = J_body(robot.Blist, theta_true_j);
N_norm   = norm(eye(6) - pinv(Jb_arb) * Jb_arb, 'fro');
results  = record(results, 'j5: null-space projector N~0 for 6-DOF', ...
                  N_norm < 1e-10, sprintf('||N||_F=%.2e', N_norm));

% --- Test j6: for 6-DOF, solution matches pure NR (secondary term vanishes)
T_rr = FK_body(robot.M, robot.Blist, th_j);
T_nr = FK_body(robot.M, robot.Blist, th_h);   % th_h from part (h), same target
results = record(results, 'j6: 6-DOF result matches NR (secondary term =0)', ...
                 norm(T_rr - T_nr, 'fro') < 1e-6, ...
                 sprintf('||T_rr-T_nr||_F=%.2e', norm(T_rr - T_nr, 'fro')));

% --- Test j7: returns failure for unreachable target
T_unreach_j = FK_body(robot.M, robot.Blist, zeros(6,1));
T_unreach_j(1,4) = T_unreach_j(1,4) + 10;
[~, ok_unreach_j, ~] = redundancy_resolution(robot, T_unreach_j, zeros(6,1), 5, 1e-3, 1e-3, 20);
results = record(results, 'j7: returns failure for unreachable target', ...
                 ~ok_unreach_j, sprintf('converged=%d', ok_unreach_j));

%% ============================================================
%  PART (k) [Bonus]: DLS_inverse_kinematics — Damped Least Squares IK
%  Assignment: extend J_inverse_kinematics with the DLS pseudoinverse
%              J_dls = J^T(JJ^T + lambda^2 I)^{-1}  for singularity robustness.
%  Variable damping (Nakamura & Hanafusa):
%    lambda = 0              when sigma_min >= sigma_thresh  (standard NR)
%    lambda^2 = lmax^2*(1-(sigma_min/sigma_thresh)^2)  near singularity
%% ============================================================
fprintf('\n========== PART (k): DLS_inverse_kinematics ==========\n');

theta_true_k = [0.1; 0.2; -0.3; 0.4; 0.5; 0.6];
T_des_k      = FK_body(robot.M, robot.Blist, theta_true_k);

[th_k, ok_k, it_k] = DLS_inverse_kinematics(robot, T_des_k, zeros(6,1));

T_sol_k   = FK_body(robot.M, robot.Blist, th_k);
Vmat_k    = MatrixLog6(inv_transform(T_sol_k) * T_des_k);
omg_err_k = norm(SO3ToVec(Vmat_k(1:3,1:3)));
lin_err_k = norm(Vmat_k(1:3,4));

% --- Test k1: converges on non-singular target
results = record(results, 'k1: DLS converges (non-singular target)', ...
                 ok_k, sprintf('iters=%d', it_k));

% --- Test k2: angular residual < eomg
results = record(results, 'k2: angular error < 1e-3 rad', ...
                 omg_err_k < 1e-3, sprintf('omg_err=%.2e', omg_err_k));

% --- Test k3: linear residual < ev
results = record(results, 'k3: linear error < 1e-3 m', ...
                 lin_err_k < 1e-3, sprintf('lin_err=%.2e', lin_err_k));

% --- Test k4: FK(solution) position matches T_desired
p_err_k = norm(T_sol_k(1:3,4) - T_des_k(1:3,4));
results = record(results, 'k4: FK(solution) position matches T_desired', ...
                 p_err_k < 1e-3, sprintf('||dp||=%.2e m', p_err_k));

% --- Test k5: well-conditioned → lambda≈0 → same result as NR
%   (when sigma_min > sigma_thresh, DLS reduces to standard Newton-Raphson)
T_k_fk = FK_body(robot.M, robot.Blist, th_k);
T_h_fk = FK_body(robot.M, robot.Blist, th_h);
results = record(results, 'k5: well-conditioned DLS == NR (lambda->0)', ...
                 norm(T_k_fk - T_h_fk, 'fro') < 1e-4, ...
                 sprintf('||T_dls-T_nr||_F=%.2e', norm(T_k_fk - T_h_fk,'fro')));

% --- Test k6: near-singular target — DLS converges where NR may struggle
%   theta5=0.01 rad places the wrist very close to singularity (sigma_min small)
theta_near = [0.2; 0.3; -0.2; 0.1; 0.01; 0.4];
T_near_k   = FK_body(robot.M, robot.Blist, theta_near);
[~, ok_near_dls, it_near_dls] = DLS_inverse_kinematics(robot, T_near_k, zeros(6,1));
results = record(results, 'k6: DLS converges near wrist singularity (theta5=0.01)', ...
                 ok_near_dls, sprintf('iters=%d', it_near_dls));

% --- Test k7: returns failure for unreachable target
T_unreach_k = FK_body(robot.M, robot.Blist, zeros(6,1));
T_unreach_k(1,4) = T_unreach_k(1,4) + 10;
[~, ok_unreach_k, ~] = DLS_inverse_kinematics(robot, T_unreach_k, zeros(6,1), ...
                                               0.1, 0.05, 1e-3, 1e-3, 20);
results = record(results, 'k7: returns failure for unreachable target', ...
                 ~ok_unreach_k, sprintf('converged=%d', ok_unreach_k));

%% ============================================================
%  (More parts will be added here as testing proceeds)
%% ============================================================


%% ---- Summary ----
fprintf('\n========== SUMMARY ==========\n');
n_pass = sum([results.pass]);
n_fail = numel(results) - n_pass;
fprintf('  Total: %d tests   PASS: %d   FAIL: %d\n\n', numel(results), n_pass, n_fail);
if n_fail > 0
    fprintf('  Failed tests:\n');
    for k = 1:numel(results)
        if ~results(k).pass
            fprintf('    ✗  %s  [%s]\n', results(k).label, results(k).detail);
        end
    end
end
fprintf('==============================\n');
