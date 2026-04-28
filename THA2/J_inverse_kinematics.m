function [thetalist, success, iter, history] = J_inverse_kinematics(robot, T_desired, thetalist0, ...
                                                                     eomg, ev, max_iter, verbose)
% J_inverse_kinematics: Iterative Newton-Raphson inverse kinematics.
%
% Starting from an initial guess thetalist0, uses the body-Jacobian
% pseudoinverse to iteratively drive the body twist error to zero:
%
%   [V_b] = log( T_b(theta)^{-1} * T_desired )
%   theta <- theta + pinv(J_b(theta)) * V_b
%
% This is Algorithm 6.2 from Lynch & Park, "Modern Robotics".
%
% Inputs:
%   robot      - robot struct from KR120_params()
%   T_desired  - 4x4 desired end-effector transform
%   thetalist0 - n x 1 initial joint angle guess (radians)
%   eomg       - angular error tolerance (rad)   (default: 1e-3)
%   ev         - linear  error tolerance (m)     (default: 1e-3)
%   max_iter   - maximum iterations              (default: 100)
%   verbose    - print convergence result        (default: true)
%
% Outputs:
%   thetalist - n x 1 solution joint angles (radians)
%   success   - true if converged within tolerances
%   iter      - number of iterations taken
%   history   - (optional) struct array of per-iteration state; requesting
%               this 4th output enables recording (see record_ik_state.m)

    if nargin < 4, eomg     = 1e-3; end
    if nargin < 5, ev       = 1e-3; end
    if nargin < 6, max_iter = 100;  end
    if nargin < 7, verbose  = true; end

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

        Jb        = J_body(robot.Blist, thetalist);
        thetalist = thetalist + pinv(Jb) * V_b;
    end

    if verbose
        if success
            fprintf('IK converged in %d iterations.\n', iter);
        else
            fprintf('IK did NOT converge after %d iterations. Final error: omg=%.4f v=%.4f\n', ...
                    max_iter, norm(V_b(1:3)), norm(V_b(4:6)));
        end
    end
end
