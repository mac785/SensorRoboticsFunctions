function [thetalist, success, iter, history] = J_transpose_kinematics(robot, T_desired, thetalist0, ...
                                                               alpha, eomg, ev, max_iter, use_adaptive, verbose)
% J_transpose_kinematics: Jacobian-transpose iterative inverse kinematics.
%
% Uses the Jacobian transpose (instead of the pseudoinverse) to update
% joint angles.  Supports both a fixed step size and an adaptive step size
% that is re-computed each iteration via an exact line search.
%
%   Fixed:    theta <- theta + alpha * J_b' * V_b
%   Adaptive: theta <- theta + alpha_k * J_b' * V_b
%             where alpha_k is chosen to minimise the linearised task error.
%
% where V_b is the body twist error computed from MatrixLog6.
%
% Inputs:
%   robot        - robot struct from KR120_params()
%   T_desired    - 4x4 desired end-effector transform
%   thetalist0   - n x 1 initial joint angle guess (radians)
%   alpha        - fixed step size used when use_adaptive=false (default: 0.1)
%   eomg         - angular error tolerance (rad)   (default: 1e-3)
%   ev           - linear  error tolerance (m)     (default: 1e-3)
%   max_iter     - maximum iterations              (default: 500)
%   use_adaptive - true to use per-iteration optimal step size (default: true)
%   verbose      - print convergence result        (default: true)
%
% Outputs:
%   thetalist - n x 1 solution joint angles (radians)
%   success   - true if converged within tolerances
%   iter      - number of iterations taken
%   history   - (optional) struct array of per-iteration state; requesting
%               this 4th output enables recording (see record_ik_state.m)

    if nargin < 4, alpha        = 0.1;  end
    if nargin < 5, eomg         = 1e-3; end
    if nargin < 6, ev           = 1e-3; end
    if nargin < 7, max_iter     = 500;  end
    if nargin < 8, use_adaptive = true; end
    if nargin < 9, verbose      = true; end

    % Hard ceiling for adaptive alpha — prevents blow-up near singularities.
    ALPHA_MAX = 10.0;

    do_record = nargout >= 4;
    history   = [];
    if do_record
        history = struct('theta',{},'T_curr',{},'omg_err',{},'lin_err',{}, ...
                         'kappa',{},'iso',{},'vol_lin',{},'vol_ang',{}, ...
                         'lambda',{},'converged',{},'elapsed_s',{});
        t_start = tic;
    end

    thetalist = thetalist0(:);
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

        Jb = J_body(robot.Blist, thetalist);

        if use_adaptive
            % Optimal step size: exact line search minimising ||V_b - alpha*J*J'*V_b||^2.
            % Buss, S.R. (2004) "Intro to IK with Jacobian Transpose, Pseudoinverse and
            % Damped Least Squares Methods", UC San Diego Tech Report, eq. 11.
            g     = Jb' * V_b;
            Jg    = Jb * g;
            denom = Jg' * Jg;
            if denom > eps
                alpha_k = min((g' * g) / denom, ALPHA_MAX);
            else
                alpha_k = ALPHA_MAX;    % near-singularity fallback
            end
        else
            alpha_k = alpha;
        end

        thetalist = thetalist + alpha_k * (Jb' * V_b);
    end

    if verbose
        if success
            fprintf('J-Transpose IK converged in %d iterations.\n', iter);
        else
            fprintf('J-Transpose IK did NOT converge after %d iterations. Final error: omg=%.4f v=%.4f\n', ...
                    max_iter, norm(V_b(1:3)), norm(V_b(4:6)));
        end
    end
end
