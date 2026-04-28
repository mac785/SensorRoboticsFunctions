%% compare_jt_alpha.m
% Compares fixed-alpha vs adaptive-alpha Jacobian Transpose IK on N random
% reachable targets.
%
% Targets are generated the same way as test_THA2 part (c3): random joint
% angles fed through FK give guaranteed-reachable poses.  Both methods
% start from the same perturbed initial guess so the comparison is fair.
%
% Outputs:
%   - Console summary table (success rate, mean/median iterations, mean errors)
%   - Figure 1: per-target iteration count, static vs adaptive side-by-side
%   - Figure 2: per-iteration error decay for one representative target
%
% Run from the THA2/tests/ directory:
%   >> compare_jt_alpha

addpath('..');
addpath('../helpers');

robot = KR120_params();

%% ---- Parameters ----
rng(42);            % fixed seed for reproducibility
N         = 30;     % number of random targets
PERTURB   = 0.15;   % std dev of initial-guess noise (rad)
ALPHA     = 0.1;    % fixed alpha for the static method
EOMG      = 1e-3;
EV        = 1e-3;
MAX_ITER  = 500;

%% ---- Generate random reachable targets ----
theta_true = (rand(6, N) - 0.5) * pi;   % random joints in [-pi/2, pi/2]
theta0_all = theta_true + PERTURB * randn(6, N);  % perturbed initial guesses

%% ---- Run both methods on every target ----
iters_s  = zeros(1, N);   ok_s = false(1, N);
omg_s    = zeros(1, N);   lin_s = zeros(1, N);
iters_a  = zeros(1, N);   ok_a = false(1, N);
omg_a    = zeros(1, N);   lin_a = zeros(1, N);

fprintf('Running %d random targets (static alpha = %.2f)...\n', N, ALPHA);
for k = 1:N
    T_des  = FK_body(robot.M, robot.Blist, theta_true(:,k));
    theta0 = theta0_all(:,k);

    [th_s, ok_s(k), iters_s(k)] = J_transpose_kinematics( ...
        robot, T_des, theta0, ALPHA, EOMG, EV, MAX_ITER, false, false);
    Vm = MatrixLog6(inv_transform(FK_body(robot.M, robot.Blist, th_s)) * T_des);
    omg_s(k) = norm(SO3ToVec(Vm(1:3,1:3)));
    lin_s(k) = norm(Vm(1:3,4));

    [th_a, ok_a(k), iters_a(k)] = J_transpose_kinematics( ...
        robot, T_des, theta0, ALPHA, EOMG, EV, MAX_ITER, true, false);
    Vm = MatrixLog6(inv_transform(FK_body(robot.M, robot.Blist, th_a)) * T_des);
    omg_a(k) = norm(SO3ToVec(Vm(1:3,1:3)));
    lin_a(k) = norm(Vm(1:3,4));
end

%% ---- Console summary ----
fprintf('\n%-24s  %-8s  %-10s  %-12s  %-12s  %-12s\n', ...
        'Method', 'Success', 'Mean iter', 'Med iter', 'Mean omg_err', 'Mean lin_err');
fprintf('%s\n', repmat('-',1,78));

ns = sum(ok_s);  na = sum(ok_a);
fprintf('%-24s  %d / %d    %-10.1f  %-12.0f  %-12.2e  %-12.2e\n', ...
        'Static  (alpha=0.1)', ns, N, mean(iters_s(ok_s)), median(iters_s(ok_s)), ...
        mean(omg_s(ok_s)), mean(lin_s(ok_s)));
fprintf('%-24s  %d / %d    %-10.1f  %-12.0f  %-12.2e  %-12.2e\n', ...
        'Adaptive (Buss 2004)', na, N, mean(iters_a(ok_a)), median(iters_a(ok_a)), ...
        mean(omg_a(ok_a)), mean(lin_a(ok_a)));

if na > 0 && ns > 0
    speedup = mean(iters_s(ok_s & ok_a)) / mean(iters_a(ok_s & ok_a));
    fprintf('\nSpeedup on targets where both converged: %.1fx fewer iterations (adaptive).\n', speedup);
end

%% ---- Figure 1: per-target iteration count ----
fig1 = figure('Name','JT Alpha Comparison — Iterations','Color','w', ...
              'Position',[100 100 900 420]);
ax1 = axes('Parent',fig1);
x   = 1:N;
bar(ax1, x, [iters_s(:), iters_a(:)], 'grouped');
legend(ax1, 'Static (\alpha=0.1)', 'Adaptive (Buss 2004)', 'Location','northeast');
xlabel(ax1, 'Target index');
ylabel(ax1, 'Iterations to convergence (MAX\_ITER if failed)');
title(ax1, 'Jacobian Transpose IK — Static vs Adaptive \alpha', 'FontWeight','bold');
set(ax1,'Color','w','XColor','k','YColor','k','GridColor',[0.82 0.82 0.82]);
grid(ax1,'on');
yline(ax1, MAX_ITER, 'r--', 'Did not converge', 'LabelHorizontalAlignment','left');

%% ---- Figure 2: per-iteration error decay (single representative target) ----
% Pick the first target where BOTH methods converged for a clean comparison.
rep = find(ok_s & ok_a, 1);
if isempty(rep)
    fprintf('\nNo target converged for both methods — skipping decay plot.\n');
else
    T_rep  = FK_body(robot.M, robot.Blist, theta_true(:,rep));
    th0_rep = theta0_all(:,rep);

    hist_s = collect_ik_history(robot, T_rep, th0_rep, 'JTStatic', ...
                'alpha', ALPHA, 'eomg', EOMG, 'ev', EV, 'max_iter', MAX_ITER);
    hist_a = collect_ik_history(robot, T_rep, th0_rep, 'JT', ...
                'eomg', EOMG, 'ev', EV, 'max_iter', MAX_ITER);

    err_s = arrayfun(@(h) norm([h.omg_err; h.lin_err]), hist_s);
    err_a = arrayfun(@(h) norm([h.omg_err; h.lin_err]), hist_a);

    fig2 = figure('Name','JT Alpha Comparison — Error Decay','Color','w', ...
                  'Position',[150 150 700 420]);
    ax2 = axes('Parent',fig2);
    semilogy(ax2, 1:numel(err_s), err_s, 'b-',  'LineWidth',1.8, 'DisplayName','Static');
    hold(ax2,'on');
    semilogy(ax2, 1:numel(err_a), err_a, 'r-',  'LineWidth',1.8, 'DisplayName','Adaptive');
    yline(ax2, sqrt(EOMG^2 + EV^2), 'k--', 'Tolerance', 'LabelHorizontalAlignment','left');
    legend(ax2, 'Location','northeast');
    xlabel(ax2, 'Iteration');
    ylabel(ax2, '||[omg\_err; lin\_err]|| (mixed units)');
    title(ax2, sprintf('Error decay — target %d  (static: %d iters, adaptive: %d iters)', ...
          rep, iters_s(rep), iters_a(rep)), 'FontWeight','bold');
    set(ax2,'Color','w','XColor','k','YColor','k','GridColor',[0.82 0.82 0.82]);
    grid(ax2,'on');
end

