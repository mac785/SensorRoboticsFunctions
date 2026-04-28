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
addpath('.')
addpath('helpers');

%% ========================================================================
%%  SECTION 1 — Configuration
%% ========================================================================

if ~exist('N_POINTS',           'var'), N_POINTS           = 20;     end
if ~exist('GOAL_THETAS',        'var'), GOAL_THETAS        = [];     end
if ~exist('RANDOM_START',       'var'), RANDOM_START       = false;  end
if ~exist('RANDOM_SEED',        'var'), RANDOM_SEED        = 42;     end
if ~exist('THRESH_ITER_RATIO',  'var'), THRESH_ITER_RATIO  = 50.0;   end  % JT is ~100x NR by design
if ~exist('THRESH_KAPPA_RATIO', 'var'), THRESH_KAPPA_RATIO = 5.0;    end
if ~exist('THRESH_TIME_RATIO',  'var'), THRESH_TIME_RATIO  = 50.0;   end  % mirrors iter ratio
if ~exist('MIN_INTERESTING_SCORE','var'), MIN_INTERESTING_SCORE = 2;  end  % require multiple flags
if ~exist('ANIMATE_INTERESTING','var'), ANIMATE_INTERESTING = false;  end
if ~exist('USE_RST',            'var'), USE_RST            = true;   end
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
% JT is excluded from convergence and ratio checks because it is a
% fundamentally slower algorithm — it failing or taking more iterations
% than NR/RR/DLS is expected behaviour, not a meaningful comparison point.
%
% Scoring per point (pi_):
%   +2  convergence mismatch among {NR, RR, DLS}
%   +2  JT converges but one or more of {NR, RR, DLS} does not  (surprising)
%   +1  any of {NR, RR, DLS} has > THRESH_KAPPA_RATIO × min kappa among that group
%   +1  any of {NR, RR, DLS} used > THRESH_ITER_RATIO  × min iters among that group
%   +1  any of {NR, RR, DLS} took > THRESH_TIME_RATIO  × min time among that group

% Indices within METHODS for the three fast methods and JT
FAST_IDX = [1, 3, 4];   % NR, RR, DLS
JT_IDX   = 2;

interest_score  = zeros(N_POINTS, 1);
interest_detail = cell(N_POINTS, 1);

for pi_ = 1:N_POINTS
    score   = 0;
    details = {};

    conv_flags = logical([results(pi_,:).converged]);
    iters_vec  = [results(pi_,:).n_iters];
    time_vec   = [results(pi_,:).elapsed_s];
    kappa_vec  = [results(pi_,:).kappa_f];

    fast_conv  = conv_flags(FAST_IDX);
    fast_iters = iters_vec(FAST_IDX);
    fast_times = time_vec(FAST_IDX);
    fast_kappa = kappa_vec(FAST_IDX);
    jt_conv    = conv_flags(JT_IDX);

    % --- Convergence mismatch among fast methods ---
    n_fast_conv = sum(fast_conv);
    if n_fast_conv > 0 && n_fast_conv < numel(FAST_IDX)
        score = score + 2;
        failed = LABELS(FAST_IDX(~fast_conv));
        details{end+1} = sprintf('Fast-method convergence mismatch: failed: %s  (+2)', ...
            strjoin(failed, ', '));
    end

    % --- Surprising: JT converges but a fast method does not ---
    if jt_conv && ~all(fast_conv)
        score = score + 2;
        failed = LABELS(FAST_IDX(~fast_conv));
        details{end+1} = sprintf('JT converged but fast method(s) failed: %s  (+2)', ...
            strjoin(failed, ', '));
    end

    % --- Iteration ratio among fast methods only ---
    min_fi = min(fast_iters);
    for k = 1:numel(FAST_IDX)
        mi = FAST_IDX(k);
        if min_fi > 0 && (fast_iters(k) / min_fi) > THRESH_ITER_RATIO
            score = score + 1;
            details{end+1} = sprintf('%s used %dx more iters than fastest fast-method  (+1)', ...
                METHODS{mi}, round(fast_iters(k) / min_fi));
        end
    end

    % --- Condition number ratio among fast methods only ---
    pos_fk = fast_kappa(fast_kappa > 0);
    if ~isempty(pos_fk)
        min_fk = min(pos_fk);
        for k = 1:numel(FAST_IDX)
            mi = FAST_IDX(k);
            if fast_kappa(k) / min_fk > THRESH_KAPPA_RATIO
                score = score + 1;
                details{end+1} = sprintf('%s kappa=%.1f  (%.1fx above min=%.1f)  (+1)', ...
                    METHODS{mi}, fast_kappa(k), fast_kappa(k)/min_fk, min_fk);
            end
        end
    end

    % --- Solve time ratio among fast methods only ---
    pos_ft = fast_times(fast_times > 0);
    if ~isempty(pos_ft)
        min_ft = min(pos_ft);
        for k = 1:numel(FAST_IDX)
            mi = FAST_IDX(k);
            if fast_times(k) / min_ft > THRESH_TIME_RATIO
                score = score + 1;
                details{end+1} = sprintf('%s time=%.4fs  (%.1fx above fastest fast-method=%.4fs)  (+1)', ...
                    METHODS{mi}, fast_times(k), fast_times(k)/min_ft, min_ft);
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
n_interesting = sum(sorted_scores >= MIN_INTERESTING_SCORE);

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

if ANIMATE_INTERESTING && n_interesting > 0  % n_interesting already respects MIN_INTERESTING_SCORE
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
            VIDEO_FILE = fullfile('media', sprintf('bench_anim_p%d_%s.avi', pi_, lower(METHOD)));

            fprintf('  Animating [%s]', METHOD);
            if RECORD_ANIMATIONS
                fprintf(' -> %s', VIDEO_FILE);
            end
            fprintf('\n');

            if USE_RST
                ik_animation_rst;
            else
                run('Archive/ik_animation.m');
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
