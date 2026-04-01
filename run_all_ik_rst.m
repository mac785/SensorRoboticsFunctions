%% run_all_ik_rst.m
% Runs all four IK methods, prints convergence statistics, and saves videos
% using the RST (Robotics System Toolbox) renderer with DAE visual meshes.
%
%   >> run_all_ik_rst

clc; close all;
addpath('.');

robot      = KR120_params();
THETA_0    = zeros(6,1);
THETA_GOAL = [pi/2; -pi/4; pi/3; pi/4; pi/4; pi/6];
T_desired  = FK_body(robot.M, robot.Blist, THETA_GOAL);

METHODS = {'NR',  'JT',  'RR',  'DLS'};
LABELS  = {'Newton-Raphson', 'Jacobian Transpose', ...
           'Redundancy Resolution', 'Damped Least-Squares'};
IK_OPTS = {
    {};
    {'alpha',0.1,'max_iter',500};
    {'k0',5};
    {'lambda_max',0.1,'sigma_thresh',0.05};
};

%% ---- Collect stats for all methods ----
histories = cell(1,4);

fprintf('\n%s\n', repmat('=',1,70));
fprintf('  IK Method Comparison  —  KUKA KR120 R2500 Pro  (RST renderer)\n');
fprintf('  Start: home (all zeros)     Goal: [pi/2, -pi/4, pi/3, pi/4, pi/4, pi/6]\n');
fprintf('%s\n\n', repmat('=',1,70));

for mi = 1:4
    fprintf('Collecting [%s] ...\n', METHODS{mi});
    histories{mi} = collect_ik_history(robot, T_desired, THETA_0, ...
                                       METHODS{mi}, IK_OPTS{mi}{:});
end

%% ---- Column legend ----
fprintf('\n%s\n', repmat('=',1,90));
fprintf('  COLUMN LEGEND\n');
fprintf('%s\n', repmat('-',1,90));
fprintf('  %-12s  %s\n', 'Iters',     'Total IK iterations before convergence or max limit');
fprintf('  %-12s  %s\n', 'Converged', 'Whether the solver reached the goal within tolerance');
fprintf('  %-12s  %s\n', 'omg_err',   'Angular error at final iteration (rad)');
fprintf('  %-12s  %s\n', 'lin_err',   'Linear position error at final iteration (m)');
fprintf('  %-12s  %s\n', 'kappa_f',   'Condition number at solution: sigma_max/sigma_min  (1=ideal, large=near-singular)');
fprintf('  %-12s  %s\n', 'Time (s)',  'Total wall-clock solve time (seconds)');
fprintf('  %-12s  %s\n', 'ms/iter',   'Average wall-clock time per iteration (milliseconds)');
fprintf('  %-12s  %s\n', 'kappa_0',   'Condition number at start (home position)');
fprintf('  %-12s  %s\n', 'iso_0/f',   'Isotropy index sigma_min/sigma_max  (0-1, 1=moves equally in all directions)');
fprintf('  %-12s  %s\n', 'vol_lin_f', 'Linear velocity ellipsoid volume at solution  (larger=more translational mobility)');
fprintf('  %-12s  %s\n', 'vol_ang_f', 'Angular velocity ellipsoid volume at solution  (larger=more rotational mobility)');
fprintf('%s\n', repmat('=',1,90));

%% ---- Print stats table ----
fprintf('\n%s\n', repmat('-',1,90));
fprintf('  %-22s  %6s  %9s  %9s  %9s  %9s  %8s  %8s\n', ...
        'Method', 'Iters', 'Converged', 'omg_err', 'lin_err', 'kappa_f', ...
        'Time (s)', 'ms/iter');
fprintf('%s\n', repmat('-',1,90));

for mi = 1:4
    h   = histories{mi};
    hf  = h(end);
    total_s     = hf.elapsed_s;
    ms_per_iter = total_s / numel(h) * 1000;
    fprintf('  %-22s  %6d  %9s  %9.2e  %9.2e  %9.1f  %8.3f  %8.3f\n', ...
            LABELS{mi}, numel(h), mat2str(hf.converged), ...
            hf.omg_err, hf.lin_err, min(hf.kappa,1e6), total_s, ms_per_iter);
end

fprintf('%s\n\n', repmat('-',1,90));

fprintf('  %-22s  %9s  %9s  %9s  %9s  %10s  %10s\n', ...
        'Method', 'kappa_0', 'kappa_f', 'iso_0', 'iso_f', 'vol_lin_f', 'vol_ang_f');
fprintf('%s\n', repmat('-',1,90));

for mi = 1:4
    h  = histories{mi};
    h0 = h(1); hf = h(end);
    fprintf('  %-22s  %9.2f  %9.2f  %9.4f  %9.4f  %10.4f  %10.4f\n', ...
            LABELS{mi}, min(h0.kappa,1e6), min(hf.kappa,1e6), ...
            h0.iso, hf.iso, hf.vol_lin, hf.vol_ang);
end

fprintf('%s\n\n', repmat('=',1,90));

%% ---- Produce videos ----
fprintf('Producing videos...\n\n');

for mi = 1:4
    METHOD = METHODS{mi};
    fprintf('--- Rendering %s ---\n', LABELS{mi});
    ik_animation_rst;
    close all;
    fprintf('\n');
end

fprintf('%s\n', repmat('=',1,70));
fprintf('  All done. Videos saved:\n');
for mi = 1:4
    fprintf('    ik_anim_rst_%s.avi\n', lower(METHODS{mi}));
end
fprintf('%s\n', repmat('=',1,70));
