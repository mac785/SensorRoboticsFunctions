function [thetalist, success, iter, history] = redundancy_resolution(robot, T_desired, thetalist0, ...
                                                             k0, eomg, ev, max_iter, verbose)
% redundancy_resolution: Null-space manipulability-maximising IK.
%
% Extends J_inverse_kinematics with a secondary objective that maximises
% the manipulability measure w = sqrt(det(Js*Js')) by projecting the
% gradient of w into the Jacobian null space.
%
% For an n-DOF robot with task dimension m < n, the null space of J_b has
% dimension (n - m), and the update is:
%
%   theta_dot = J_b^+ * V_b  +  (I - J_b^+ * J_b) * (k0 * grad_w)
%   \_______/   \___________/    \________________/   \__________/
%   full update  primary task     null-space proj.   manip. gradient
%
% For a 6-DOF robot (n = m = 6) the null space is trivial, so the secondary
% term vanishes and the function reduces to standard Newton-Raphson IK.
% It is fully effective for redundant robots (n > 6).
%
% Inputs:
%   robot      - robot struct from KR120_params()
%   T_desired  - 4x4 desired end-effector transform
%   thetalist0 - n x 1 initial joint angle guess (radians)
%   k0         - secondary-task gain (default: 5)
%   eomg       - angular error tolerance (rad)   (default: 1e-3)
%   ev         - linear  error tolerance (m)     (default: 1e-3)
%   max_iter   - maximum iterations              (default: 200)
%   verbose    - print convergence result        (default: true)
%
% Outputs:
%   thetalist - n x 1 solution joint angles (radians)
%   success   - true if converged within tolerances
%   iter      - number of iterations taken
%   history   - (optional) struct array of per-iteration state; requesting
%               this 4th output enables recording (see record_ik_state.m)

    if nargin < 4, k0       = 5;    end
    if nargin < 5, eomg     = 1e-3; end
    if nargin < 6, ev       = 1e-3; end
    if nargin < 7, max_iter = 200;  end
    if nargin < 8, verbose  = true; end

    do_record = nargout >= 4;
    history   = [];
    if do_record
        history = struct('theta',{},'T_curr',{},'omg_err',{},'lin_err',{}, ...
                         'kappa',{},'iso',{},'vol_lin',{},'vol_ang',{}, ...
                         'lambda',{},'converged',{},'elapsed_s',{});
        t_start = tic;
    end

    thetalist = thetalist0(:);
    n         = numel(thetalist);
    success   = false;
    V_b       = zeros(6,1);

    for iter = 1:max_iter
        T_curr  = FK_body(robot.M, robot.Blist, thetalist);
        V_b_mat = MatrixLog6(inv_transform(T_curr) * T_desired);
        V_b     = [SO3ToVec(V_b_mat(1:3,1:3)); V_b_mat(1:3,4)];

        converged_now = norm(V_b(1:3)) < eomg && norm(V_b(4:6)) < ev;
        if do_record
            s = record_ik_state(robot, thetalist, T_curr, V_b, 0, t_start);
            s.converged = converged_now;
            history(end+1) = s; %#ok<AGROW>
        end

        if converged_now
            success = true;
            break;
        end

        Jb      = J_body(robot.Blist, thetalist);
        Jb_pinv = pinv(Jb);

        %% Secondary task: gradient of manipulability w.r.t. theta
        % Compute numerically:  grad_w(i) = (w(theta + eps*e_i) - w(theta)) / eps
        eps_fd = 1e-6;
        Js0    = J_space(robot.Slist, thetalist);
        w0     = sqrt(abs(det(Js0 * Js0')));
        grad_w = zeros(n, 1);
        for i = 1:n
            th_p       = thetalist;
            th_p(i)    = th_p(i) + eps_fd;
            Js_p       = J_space(robot.Slist, th_p);
            w_p        = sqrt(abs(det(Js_p * Js_p')));
            grad_w(i)  = (w_p - w0) / eps_fd;
        end

        %% Null-space projection
        N = eye(n) - Jb_pinv * Jb;

        %% Combined update
        thetalist = thetalist + Jb_pinv * V_b + N * (k0 * grad_w);
    end

    if verbose
        if success
            fprintf('Redundancy-resolution IK converged in %d iterations.\n', iter);
        else
            fprintf('Redundancy-resolution IK did NOT converge after %d iterations. Final error: omg=%.4f v=%.4f\n', ...
                    max_iter, norm(V_b(1:3)), norm(V_b(4:6)));
        end
    end
end
