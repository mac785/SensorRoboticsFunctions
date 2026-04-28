%% animate_benchmark_result.m
% Animates a specific (point, method) pair from a previous benchmark_ik run.
%
% REQUIREMENTS:
%   Run benchmark_ik first — this script reads variables left in the workspace:
%     sort_idx      Interesting point indices sorted by score (descending)
%     GOAL_THETAS   N×6 matrix of goal joint angles used in the benchmark
%     START_THETAS  N×6 matrix of start joint angles used in the benchmark
%
% USAGE:
%   1. Run benchmark_ik to populate the workspace.
%   2. Read the interesting-run report printed in the terminal.
%   3. Edit RANK and METHOD below, then run this script.
%
%   >> benchmark_ik
%   >> animate_benchmark_result

%% ========================================================================
%%  CONFIGURATION — edit these two lines
%% ========================================================================

RANK   = 1;     % which interesting run to animate:
                %   1 = most interesting (highest score)
                %   2 = second most interesting, etc.
                %   Use the printed report to decide which rank to inspect.

METHOD = 'RR';  % which IK method to animate:
                %   'NR'  — Newton-Raphson
                %   'JT'  — Jacobian Transpose
                %   'RR'  — Redundancy Resolution
                %   'DLS' — Damped Least-Squares

%% ========================================================================
%%  ANIMATION OPTIONS
%% ========================================================================

USE_RST = true;   % true  = RST renderer (DAE meshes, requires load_kr120_rst)
                  % false = skeleton + STL renderer

RECORD     = true;                % true = save an .avi file
VIDEO_FILE = sprintf('bench_anim_rank%d_%s.avi', RANK, lower(METHOD));
                                   % filename used when RECORD = true

%% ========================================================================
%%  RESOLVE POINT INDEX FROM RANK
%% ========================================================================

% sort_idx is produced by benchmark_ik and holds point indices ordered
% from most to least interesting. RANK=1 is the most flagged point.
pi_        = sort_idx(RANK);

THETA_0    = START_THETAS(pi_,:)';   % start configuration for this point
THETA_GOAL = GOAL_THETAS(pi_,:)';    % goal configuration for this point

fprintf('Animating rank=%d  (point %d)  method=%s\n', RANK, pi_, METHOD);
fprintf('  Start: [%s] rad\n', num2str(THETA_0', '%.3f  '));
fprintf('  Goal:  [%s] rad\n', num2str(THETA_GOAL', '%.3f  '));

%% ========================================================================
%%  RUN ANIMATION
%% ========================================================================

if USE_RST
    ik_animation_rst;
else
    ik_animation;
end
