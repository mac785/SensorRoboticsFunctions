function [q_hist, p_tip_hist, info_hist] = simulate_VF(robot, q0, p_goal, n_steps, opts)
% simulate_VF  Run QP virtual fixture control loop to convergence.
%
% At each step calls QP_step_VF, applies the resulting joint displacement,
% and records the full trajectory.  Stops early if the step magnitude falls
% below a convergence threshold (the QP has nothing left to do).
%
% Inputs:
%   robot   - robot struct from KR120_params
%   q0      - n-by-1 initial joint angles (radians)
%   p_goal  - 3-by-1 goal position for tool tip (meters)
%   n_steps - maximum number of QP steps      (default: 800)
%   opts    - options struct forwarded to QP_step_VF, plus:
%               .conv_tol  : stop when ||dq||_inf < conv_tol  (default: 1e-6)
%               .stop_dist : stop when dist to goal < this   (default: 5e-5 m)
%               .verbose   : print progress every N steps    (default: 0)
%
% Outputs:
%   q_hist      - n-by-(K+1) joint angle history (K = steps actually taken)
%   p_tip_hist  - 3-by-(K+1) tool tip position history
%   info_hist   - 1-by-K struct array, one entry per step (from QP_step_VF)

    if nargin < 4 || isempty(n_steps), n_steps = 800; end
    if nargin < 5, opts = struct(); end

    conv_tol  = opt(opts, 'conv_tol',  1e-6);
    stop_dist = opt(opts, 'stop_dist', 5e-5);   % stop when dist < 0.05 mm
    verbose   = opt(opts, 'verbose',   0);

    n = robot.n_dof;

    % Pre-allocate (trim at end if early convergence)
    q_hist     = zeros(n, n_steps + 1);
    p_tip_hist = zeros(3, n_steps + 1);
    info_hist  = repmat(struct('dist', 0, 'exitflag', 0, ...
                               'n_constraints', 0, 'v_d', zeros(3,1)), ...
                        1, n_steps);

    q = q0(:);
    q_hist(:,    1) = q;
    p_tip_hist(:,1) = tool_tip_fk(robot, q);

    k_end = n_steps;   % steps actually taken
    for k = 1:n_steps
        [dq, info] = QP_step_VF(robot, q, p_goal, opts);
        q = q + dq;

        q_hist(:,    k+1) = q;
        p_tip_hist(:,k+1) = tool_tip_fk(robot, q);
        info_hist(k)      = info;

        if verbose > 0 && mod(k, verbose) == 0
            fprintf('Step %4d | dist=%.3f mm | ||dq||=%.2e | n_con=%d\n', ...
                k, info.dist*1e3, norm(dq, inf), info.n_constraints);
        end

        % Convergence: tiny step OR close enough to goal
        if norm(dq, inf) < conv_tol || info.dist < stop_dist
            k_end = k;
            break;
        end

        % No-progress: tip hasn't moved in the last 30 steps (wall-blocked)
        if k > 30 && norm(p_tip_hist(:,k+1) - p_tip_hist(:,k-29)) < 1e-4
            k_end = k;
            break;
        end
    end

    % Trim to actual length
    q_hist     = q_hist(:,     1:k_end+1);
    p_tip_hist = p_tip_hist(:, 1:k_end+1);
    info_hist  = info_hist(    1:k_end);
end

% -------------------------------------------------------------------------
function v = opt(s, field, default)
    if isstruct(s) && isfield(s, field)
        v = s.(field);
    else
        v = default;
    end
end
