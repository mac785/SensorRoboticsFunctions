function history = collect_ik_history(robot, T_desired, theta0, method, varargin)
% collect_ik_history: Run an IK solver and return full per-iteration history.
%
% Thin wrapper around the standalone IK functions. Calls the appropriate
% solver with history recording enabled (4th output) and verbose=false,
% then prints a single convergence summary line.
%
% Inputs:
%   robot     - robot struct from KR120_params()
%   T_desired - 4x4 desired end-effector transform
%   theta0    - 6x1 initial joint angle guess (radians)
%   method    - 'NR' | 'JT' | 'JTStatic' | 'RR' | 'DLS'
%              'JT' uses adaptive step size (Buss 2004) by default.
%              'JTStatic' uses a fixed alpha step size.
%
% Name-value options:
%   'eomg'         angular convergence tolerance  (default: 1e-3)
%   'ev'           linear  convergence tolerance  (default: 1e-3)
%   'max_iter'     maximum iterations             (default: 200)
%   'alpha'        step size           [JT only]  (default: 0.1)
%   'k0'           manipulability gain [RR only]  (default: 5)
%   'lambda_max'   max damping factor  [DLS only] (default: 0.1)
%   'sigma_thresh' singularity threshold [DLS]    (default: 0.05)
%
% Output:
%   history - struct array, one entry per iteration (state BEFORE the update):
%     .theta     6x1  joint angles (rad)
%     .T_curr    4x4  current FK transform
%     .omg_err   scalar  angular body-twist error norm (rad)
%     .lin_err   scalar  linear  body-twist error norm (m)
%     .kappa     scalar  Jacobian condition number (capped at 1e8)
%     .iso       scalar  isotropy index in [0,1]
%     .vol_lin   scalar  linear  velocity ellipsoid volume
%     .vol_ang   scalar  angular velocity ellipsoid volume
%     .lambda    scalar  DLS damping factor (0 for NR/JT/RR)
%     .converged logical  true only on the final entry if within tolerance
%     .elapsed_s scalar  wall-clock time since solve start (seconds)

    p = inputParser();
    addParameter(p, 'eomg',         1e-3);
    addParameter(p, 'ev',           1e-3);
    addParameter(p, 'max_iter',     200);
    addParameter(p, 'alpha',        0.1);
    addParameter(p, 'k0',           5);
    addParameter(p, 'lambda_max',   0.1);
    addParameter(p, 'sigma_thresh', 0.05);
    parse(p, varargin{:});
    o = p.Results;

    switch upper(method)
        case 'NR'
            [~,~,~, history] = J_inverse_kinematics( ...
                robot, T_desired, theta0, o.eomg, o.ev, o.max_iter, false);
        case 'JT'
            [~,~,~, history] = J_transpose_kinematics( ...
                robot, T_desired, theta0, o.alpha, o.eomg, o.ev, o.max_iter, true, false);
        case 'JTSTATIC'
            [~,~,~, history] = J_transpose_kinematics( ...
                robot, T_desired, theta0, o.alpha, o.eomg, o.ev, o.max_iter, false, false);
        case 'RR'
            [~,~,~, history] = redundancy_resolution( ...
                robot, T_desired, theta0, o.k0, o.eomg, o.ev, o.max_iter, false);
        case 'DLS'
            [~,~,~, history] = DLS_inverse_kinematics( ...
                robot, T_desired, theta0, o.lambda_max, o.sigma_thresh, o.eomg, o.ev, o.max_iter, false);
        otherwise
            error('collect_ik_history: unknown method ''%s''. Use NR, JT, JTStatic, RR, or DLS.', method);
    end

    if history(end).converged
        fprintf('[%s] Converged in %d iterations.\n', upper(method), numel(history));
    else
        fprintf('[%s] Did NOT converge after %d iterations. Final err: omg=%.4f v=%.4f\n', ...
                upper(method), numel(history), history(end).omg_err, history(end).lin_err);
    end
end
