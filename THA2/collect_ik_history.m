function history = collect_ik_history(robot, T_desired, theta0, method, varargin)
% collect_ik_history: Run an IK algorithm step-by-step and record metrics.
%
% Executes the chosen IK method one iteration at a time, capturing the full
% robot state and manipulability metrics at every step. Use the returned
% history struct to drive ik_animation.m or any other analysis.
%
% Inputs:
%   robot     - robot struct from KR120_params()
%   T_desired - 4x4 desired end-effector transform
%   theta0    - 6x1 initial joint angle guess (radians)
%   method    - 'NR' | 'JT' | 'RR' | 'DLS'
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
%     .kappa     scalar  Jacobian condition number (capped at 1e8 for Inf)
%     .iso       scalar  isotropy index in [0,1]
%     .vol_lin   scalar  linear  velocity ellipsoid volume
%     .vol_ang   scalar  angular velocity ellipsoid volume
%     .lambda    scalar  DLS damping factor (0 for NR/JT/RR)
%     .converged logical  true only on the final entry if within tolerance

    %% --- Parse options ---
    p = inputParser();
    addParameter(p, 'eomg',         1e-3);
    addParameter(p, 'ev',           1e-3);
    addParameter(p, 'max_iter',     200);
    addParameter(p, 'alpha',        0.1);
    addParameter(p, 'k0',           5);
    addParameter(p, 'lambda_max',   0.1);
    addParameter(p, 'sigma_thresh', 0.05);
    parse(p, varargin{:});
    opt = p.Results;

    thetalist = theta0(:);
    n         = numel(thetalist);
    history   = struct('theta',{},'T_curr',{},'omg_err',{},'lin_err',{}, ...
                       'kappa',{},'iso',{},'vol_lin',{},'vol_ang',{}, ...
                       'lambda',{},'converged',{},'elapsed_s',{});

    tic;   % start wall-clock timer before the solve loop
    for iter = 1:opt.max_iter

        %% Current FK and body-twist error
        T_curr  = FK_body(robot.M, robot.Blist, thetalist);
        V_b_mat = MatrixLog6(inv_transform(T_curr) * T_desired);
        V_b     = [SO3ToVec(V_b_mat(1:3,1:3)); V_b_mat(1:3,4)];
        omg_err = norm(V_b(1:3));
        lin_err = norm(V_b(4:6));

        %% Manipulability metrics from space Jacobian (computed inline — no fprintf)
        Js     = J_space(robot.Slist, thetalist);
        sigma  = svd(Js);
        if sigma(end) < 1e-10
            kappa = 1e8;
            iso   = 0;
        else
            kappa = sigma(1) / sigma(end);
            iso   = sigma(end) / sigma(1);
        end
        Jw      = Js(1:3,:);  Jv = Js(4:6,:);
        sigma_w = svd(Jw);    sigma_v = svd(Jv);
        vol_ang = (4/3) * pi * prod(sigma_w(1:3));
        vol_lin = (4/3) * pi * prod(sigma_v(1:3));

        %% Record state at this iteration
        s.theta     = thetalist;
        s.T_curr    = T_curr;
        s.omg_err   = omg_err;
        s.lin_err   = lin_err;
        s.kappa     = min(kappa, 1e8);
        s.iso       = iso;
        s.vol_lin   = vol_lin;
        s.vol_ang   = vol_ang;
        s.lambda    = 0;
        s.converged = (omg_err < opt.eomg && lin_err < opt.ev);
        s.elapsed_s = toc;
        history(end+1) = s; %#ok<AGROW>

        if s.converged
            fprintf('[%s] Converged in %d iterations.\n', upper(method), iter);
            return;
        end

        %% Joint update — method-specific
        Jb = J_body(robot.Blist, thetalist);

        switch upper(method)

            case 'NR'
                thetalist = thetalist + pinv(Jb) * V_b;

            case 'JT'
                thetalist = thetalist + opt.alpha * (Jb' * V_b);

            case 'RR'
                Jb_pinv = pinv(Jb);
                % Numerical gradient of manipulability
                Js0    = J_space(robot.Slist, thetalist);
                w0     = sqrt(abs(det(Js0 * Js0')));
                grad_w = zeros(n, 1);
                eps_fd = 1e-6;
                for i = 1:n
                    th_p    = thetalist; th_p(i) = th_p(i) + eps_fd;
                    Js_p    = J_space(robot.Slist, th_p);
                    grad_w(i) = (sqrt(abs(det(Js_p*Js_p'))) - w0) / eps_fd;
                end
                N = eye(n) - Jb_pinv * Jb;
                thetalist = thetalist + Jb_pinv * V_b + N * (opt.k0 * grad_w);

            case 'DLS'
                sigma_min = min(svd(Jb));
                if sigma_min >= opt.sigma_thresh
                    lambda2 = 0;
                else
                    lambda2 = opt.lambda_max^2 * (1 - (sigma_min/opt.sigma_thresh)^2);
                end
                history(end).lambda = sqrt(lambda2);
                thetalist = thetalist + Jb' * ((Jb*Jb' + lambda2*eye(6)) \ V_b);

            otherwise
                error('collect_ik_history: unknown method ''%s''. Use NR, JT, RR, or DLS.', method);
        end
    end

    fprintf('[%s] Did NOT converge after %d iterations. Final err: omg=%.4f v=%.4f\n', ...
            upper(method), opt.max_iter, history(end).omg_err, history(end).lin_err);
end
