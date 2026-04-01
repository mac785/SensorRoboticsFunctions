%% benchmark_ik.m
% Benchmarks all four IK methods across N goal configurations and reports
% aggregate statistics. Automatically flags "interesting" runs where methods
% perform notably differently from each other.
%
% CONFIG VARIABLES — set before running to override defaults:
%
%   N_POINTS           Number of goal points to test           (default: 5)
%   GOAL_THETAS        N×6 matrix of goal joint angles in rad,
%                      or [] to generate randomly              (default: [])
%   RANDOM_START       true = random THETA_0 per run,
%                      false = home (all zeros)                (default: false)
%   RANDOM_SEED        RNG seed for reproducibility            (default: 42)
%   THRESH_ITER_RATIO  Flag if any method used > X × min iters (default: 3.0)
%   THRESH_KAPPA_RATIO Flag if any method has > X × min kappa  (default: 5.0)
%   THRESH_TIME_RATIO  Flag if any method took > X × min time  (default: 5.0)
%   ANIMATE_INTERESTING Animate all 4 methods for each flagged point
%                                                              (default: false)
%   USE_RST            Use RST renderer (ik_animation_rst) for animation
%                                                              (default: false)
%   RECORD_ANIMATIONS  Save .avi files during animation        (default: false)
%
% EXAMPLES:
%   >> benchmark_ik
%   >> N_POINTS = 10; RANDOM_START = true; benchmark_ik
%   >> GOAL_THETAS = [pi/2 -pi/4 pi/3 pi/4 pi/4 pi/6]; benchmark_ik
%   >> ANIMATE_INTERESTING = true; USE_RST = true; benchmark_ik

clc; close all;
addpath('.');

%% ========================================================================
%%  SECTION 1 — Configuration
%% ========================================================================

if ~exist('N_POINTS',           'var'), N_POINTS           = 5;     end
if ~exist('GOAL_THETAS',        'var'), GOAL_THETAS        = [];     end
if ~exist('RANDOM_START',       'var'), RANDOM_START       = false;  end
if ~exist('RANDOM_SEED',        'var'), RANDOM_SEED        = 42;     end
if ~exist('THRESH_ITER_RATIO',  'var'), THRESH_ITER_RATIO  = 3.0;    end
if ~exist('THRESH_KAPPA_RATIO', 'var'), THRESH_KAPPA_RATIO = 5.0;    end
if ~exist('THRESH_TIME_RATIO',  'var'), THRESH_TIME_RATIO  = 5.0;    end
if ~exist('ANIMATE_INTERESTING','var'), ANIMATE_INTERESTING = false;  end
if ~exist('USE_RST',            'var'), USE_RST            = false;  end
if ~exist('RECORD_ANIMATIONS',  'var'), RECORD_ANIMATIONS  = false;  end

rng(RANDOM_SEED);

%% ========================================================================
%%  SECTION 2 — Robot model and method table
%% ========================================================================

robot = KR120_params();

METHODS  = {'NR',  'JT',  'RR',  'DLS'};
LABELS   = {'Newton-Raphson', 'Jacobian Transpose', ...
            'Redundancy Resolution', 'Damped Least-Squares'};
IK_OPTS  = {
    {};
    {'alpha', 0.1, 'max_iter', 3000};
    {'k0', 5};
    {'lambda_max', 0.1, 'sigma_thresh', 0.05};
};
N_METHODS = numel(METHODS);

%% ========================================================================
%%  SECTION 3 — Goal point and start configuration generation
%% ========================================================================

% Conservative joint limits for random sampling  [lower, upper]  (radians)
J_LIM = [ -3.0,  3.0;   % J1  ±~172°
          -2.1,  0.5;   % J2  biased downward like a real robot
          -0.9,  2.4;   % J3
          -3.0,  3.0;   % J4
          -2.1,  2.1;   % J5
          -3.0,  3.0 ]; % J6

if isempty(GOAL_THETAS)
    GOAL_THETAS = zeros(N_POINTS, 6);
    for pi_ = 1:N_POINTS
        for ji = 1:6
            GOAL_THETAS(pi_, ji) = J_LIM(ji,1) + rand() * diff(J_LIM(ji,:));
        end
    end
    fprintf('Generated %d random goal configurations (seed=%d).\n', N_POINTS, RANDOM_SEED);
else
    N_POINTS = size(GOAL_THETAS, 1);
    fprintf('Using %d provided goal configurations.\n', N_POINTS);
end

% Compute SE(3) goal transforms from joint angles
T_GOALS = cell(N_POINTS, 1);
for pi_ = 1:N_POINTS
    T_GOALS{pi_} = FK_body(robot.M, robot.Blist, GOAL_THETAS(pi_,:)');
end

% Start configurations
if RANDOM_START
    START_THETAS = zeros(N_POINTS, 6);
    for pi_ = 1:N_POINTS
        for ji = 1:6
            START_THETAS(pi_, ji) = J_LIM(ji,1) + rand() * diff(J_LIM(ji,:));
        end
    end
    fprintf('Using random start configurations.\n');
else
    START_THETAS = zeros(N_POINTS, 6);   % home (all joints zero)
    fprintf('Using home position (all zeros) as start for all runs.\n');
end

%% ========================================================================
%%  SECTION 4 — Run 4 × N_POINTS simulations
%% ========================================================================

fprintf('\n%s\n', repmat('=',1,72));
fprintf('  Running %d methods × %d points = %d simulations\n', ...
        N_METHODS, N_POINTS, N_METHODS * N_POINTS);
fprintf('%s\n\n', repmat('=',1,72));

% results(pi_, mi) — one struct per (point, method) pair
results(N_POINTS, N_METHODS) = struct( ...
    'converged',0, 'n_iters',0, 'elapsed_s',0, ...
    'omg_err',0, 'lin_err',0, ...
    'kappa_f',0, 'iso_f',0, 'vol_lin_f',0, 'vol_ang_f',0);

for pi_ = 1:N_POINTS
    fprintf('  Point %d / %d\n', pi_, N_POINTS);
    theta_0   = START_THETAS(pi_,:)';
    T_desired = T_GOALS{pi_};

    for mi = 1:N_METHODS
        fprintf('    [%-4s] ... ', METHODS{mi});
        h  = collect_ik_history(robot, T_desired, theta_0, ...
                                METHODS{mi}, IK_OPTS{mi}{:});
        hf = h(end);
        results(pi_, mi).converged  = hf.converged;
        results(pi_, mi).n_iters    = numel(h);
        results(pi_, mi).elapsed_s  = hf.elapsed_s;
        results(pi_, mi).omg_err    = hf.omg_err;
        results(pi_, mi).lin_err    = hf.lin_err;
        results(pi_, mi).kappa_f    = min(hf.kappa, 1e6);
        results(pi_, mi).iso_f      = hf.iso;
        results(pi_, mi).vol_lin_f  = hf.vol_lin;
        results(pi_, mi).vol_ang_f  = hf.vol_ang;
        if hf.converged
            fprintf('converged in %3d iters  (%.3f s)\n', numel(h), hf.elapsed_s);
        else
            fprintf('FAILED     in %3d iters  omg=%.2e  v=%.2e\n', ...
                    numel(h), hf.omg_err, hf.lin_err);
        end
    end
    fprintf('\n');
end

%% ========================================================================
%%  SECTION 5 — Aggregate statistics
%% ========================================================================

fprintf('\n%s\n', repmat('=',1,92));
fprintf('  AGGREGATE STATISTICS  (%d goal points)\n', N_POINTS);
fprintf('%s\n', repmat('-',1,92));
fprintf('  %-22s  %7s  %9s  %9s  %10s  %8s  %10s  %10s\n', ...
        'Method', 'Conv%', 'MeanIter', 'MeanTime', 'MeanKappa', ...
        'MeanIso', 'MeanVolLin', 'MeanVolAng');
fprintf('%s\n', repmat('-',1,92));

for mi = 1:N_METHODS
    r         = results(:, mi);
    conv_mask = logical([r.converged]);
    conv_pct  = 100 * mean(conv_mask);

    if any(conv_mask)
        rc       = r(conv_mask);
        mu_iter  = mean([rc.n_iters]);
        mu_time  = mean([rc.elapsed_s]);
        mu_kap   = mean([rc.kappa_f]);
        mu_iso   = mean([rc.iso_f]);
        mu_vlin  = mean([rc.vol_lin_f]);
        mu_vang  = mean([rc.vol_ang_f]);
    else
        [mu_iter, mu_time, mu_kap, mu_iso, mu_vlin, mu_vang] = deal(nan);
    end

    fprintf('  %-22s  %6.1f%%  %9.1f  %9.4f  %10.2f  %8.4f  %10.4f  %10.4f\n', ...
            LABELS{mi}, conv_pct, mu_iter, mu_time, mu_kap, mu_iso, mu_vlin, mu_vang);
end

fprintf('%s\n\n', repmat('=',1,92));

%% ========================================================================
%%  SECTION 6 — Interesting run detection
%% ========================================================================
% Scoring per point (pi_):
%   +2  convergence mismatch  (some converged, some did not)
%   +1  any method used > THRESH_ITER_RATIO  × min iterations
%   +1  any method has  > THRESH_KAPPA_RATIO × min condition number
%   +1  any method took > THRESH_TIME_RATIO  × min wall-clock time

interest_score  = zeros(N_POINTS, 1);
interest_detail = cell(N_POINTS, 1);

for pi_ = 1:N_POINTS
    score   = 0;
    details = {};

    conv_flags = logical([results(pi_,:).converged]);
    iters_vec  = [results(pi_,:).n_iters];
    time_vec   = [results(pi_,:).elapsed_s];
    kappa_vec  = [results(pi_,:).kappa_f];

    % --- Convergence mismatch ---
    n_conv = sum(conv_flags);
    if n_conv > 0 && n_conv < N_METHODS
        score = score + 2;
        failed_labels = LABELS(~conv_flags);
        details{end+1} = sprintf('Convergence mismatch: %d/%d converged — failed: %s  (+2)', ...
            n_conv, N_METHODS, strjoin(failed_labels, ', '));
    end

    % --- Iteration ratio ---
    min_iters = min(iters_vec);
    for mi = 1:N_METHODS
        if min_iters > 0 && (iters_vec(mi) / min_iters) > THRESH_ITER_RATIO
            score = score + 1;
            details{end+1} = sprintf('%s used %dx more iterations than fastest  (+1)', ...
                METHODS{mi}, round(iters_vec(mi) / min_iters));
        end
    end

    % --- Condition number ratio ---
    pos_kappas = kappa_vec(kappa_vec > 0);
    if ~isempty(pos_kappas)
        min_kap = min(pos_kappas);
        for mi = 1:N_METHODS
            if kappa_vec(mi) / min_kap > THRESH_KAPPA_RATIO
                score = score + 1;
                details{end+1} = sprintf('%s kappa=%.1f  (%.1fx above min=%.1f)  (+1)', ...
                    METHODS{mi}, kappa_vec(mi), kappa_vec(mi)/min_kap, min_kap);
            end
        end
    end

    % --- Solve time ratio ---
    pos_times = time_vec(time_vec > 0);
    if ~isempty(pos_times)
        min_time = min(pos_times);
        for mi = 1:N_METHODS
            if time_vec(mi) / min_time > THRESH_TIME_RATIO
                score = score + 1;
                details{end+1} = sprintf('%s time=%.4fs  (%.1fx above fastest=%.4fs)  (+1)', ...
                    METHODS{mi}, time_vec(mi), time_vec(mi)/min_time, min_time);
            end
        end
    end

    interest_score(pi_)  = score;
    interest_detail{pi_} = details;
end

%% ========================================================================
%%  SECTION 7 — Interesting run report
%% ========================================================================

[sorted_scores, sort_idx] = sort(interest_score, 'descend');
n_interesting = sum(sorted_scores > 0);

fprintf('%s\n', repmat('=',1,92));
fprintf('  INTERESTING RUNS  (%d / %d points flagged)\n', n_interesting, N_POINTS);
fprintf('%s\n', repmat('=',1,92));

if n_interesting == 0
    fprintf('  No anomalies detected. All methods behaved consistently.\n');
else
    for rank = 1:n_interesting
        pi_   = sort_idx(rank);
        sc    = sorted_scores(rank);

        fprintf('\n  Point %d  [score=%d]   goal theta: [%s] rad\n', ...
                pi_, sc, num2str(GOAL_THETAS(pi_,:), '%.3f  '));
        if RANDOM_START
            fprintf('                        start theta: [%s] rad\n', ...
                    num2str(START_THETAS(pi_,:), '%.3f  '));
        end
        fprintf('  %s\n', repmat('-',1,88));
        fprintf('  %-4s  %-9s  %6s  %9s  %10s  %8s  %9s  %9s\n', ...
                'Meth', 'Converged', 'Iters', 'Time(s)', ...
                'kappa_f', 'iso_f', 'vol_lin', 'vol_ang');
        fprintf('  %s\n', repmat('-',1,88));

        for mi = 1:N_METHODS
            r = results(pi_, mi);
            fprintf('  %-4s  %-9s  %6d  %9.4f  %10.2f  %8.4f  %9.4f  %9.4f\n', ...
                    METHODS{mi}, mat2str(r.converged), r.n_iters, r.elapsed_s, ...
                    r.kappa_f, r.iso_f, r.vol_lin_f, r.vol_ang_f);
        end

        fprintf('\n  Why flagged:\n');
        for di = 1:numel(interest_detail{pi_})
            fprintf('    * %s\n', interest_detail{pi_}{di});
        end
    end
end

fprintf('\n%s\n\n', repmat('=',1,92));

%% ========================================================================
%%  SECTION 8 — Optional animation of interesting runs
%% ========================================================================

if ANIMATE_INTERESTING && n_interesting > 0
    fprintf('Animating all 4 methods for each of the %d interesting points...\n\n', ...
            n_interesting);

    for rank = 1:n_interesting
        pi_ = sort_idx(rank);
        fprintf('--- Point %d (score=%d) ---\n', pi_, sorted_scores(rank));

        for mi = 1:N_METHODS
            METHOD     = METHODS{mi};
            THETA_0    = START_THETAS(pi_,:)';
            THETA_GOAL = GOAL_THETAS(pi_,:)';
            RECORD     = RECORD_ANIMATIONS;
            VIDEO_FILE = sprintf('bench_anim_p%d_%s.avi', pi_, lower(METHOD));

            fprintf('  Animating [%s]', METHOD);
            if RECORD_ANIMATIONS
                fprintf(' -> %s', VIDEO_FILE);
            end
            fprintf('\n');

            if USE_RST
                ik_animation_rst;
            else
                ik_animation;
            end
            close all;
        end
        fprintf('\n');
    end

    % Clean up injected variables so the workspace is tidy afterward
    clear METHOD THETA_0 THETA_GOAL RECORD VIDEO_FILE

    if RECORD_ANIMATIONS
        fprintf('Videos saved:\n');
        for rank = 1:n_interesting
            pi_ = sort_idx(rank);
            for mi = 1:N_METHODS
                fprintf('  bench_anim_p%d_%s.avi\n', pi_, lower(METHODS{mi}));
            end
        end
    end
end

fprintf('benchmark_ik complete.\n');
