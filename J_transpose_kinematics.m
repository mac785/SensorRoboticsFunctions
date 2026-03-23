function [thetalist, success, iter] = J_transpose_kinematics(robot, T_desired, thetalist0, ...
                                                               alpha, eomg, ev, max_iter)
% J_transpose_kinematics: Jacobian-transpose iterative inverse kinematics.
%
% Uses the Jacobian transpose (instead of the pseudoinverse) to update
% joint angles.  Computationally cheaper than Newton-Raphson but converges
% more slowly and may require smaller step sizes near singularities.
%
%   theta <- theta + alpha * J_b(theta)' * V_b
%
% where V_b is the body twist error computed from MatrixLog6.
%
% Inputs:
%   robot      - robot struct from KR210_params()
%   T_desired  - 4x4 desired end-effector transform
%   thetalist0 - n x 1 initial joint angle guess (radians)
%   alpha      - step size (default: 0.1)
%   eomg       - angular error tolerance (rad)   (default: 1e-3)
%   ev         - linear  error tolerance (m)     (default: 1e-3)
%   max_iter   - maximum iterations              (default: 500)
%
% Outputs:
%   thetalist - n x 1 solution joint angles (radians)
%   success   - true if converged within tolerances
%   iter      - number of iterations taken

    if nargin < 4, alpha    = 0.1;  end
    if nargin < 5, eomg     = 1e-3; end
    if nargin < 6, ev       = 1e-3; end
    if nargin < 7, max_iter = 500;  end

    thetalist = thetalist0(:);
    success   = false;
    V_b       = zeros(6,1);   % initialise for the post-loop error report

    for iter = 1:max_iter
        T_curr  = FK_body(robot.M, robot.Blist, thetalist);
        V_b_mat = MatrixLog6(inv_transform(T_curr) * T_desired);
        V_b     = [SO3ToVec(V_b_mat(1:3,1:3)); V_b_mat(1:3,4)];

        if norm(V_b(1:3)) < eomg && norm(V_b(4:6)) < ev
            success = true;
            break;
        end

        Jb        = J_body(robot.Blist, thetalist);
        thetalist = thetalist + alpha * (Jb' * V_b);
    end

    if success
        fprintf('J-Transpose IK converged in %d iterations.\n', iter);
    else
        fprintf('J-Transpose IK did NOT converge after %d iterations. Final error: omg=%.4f v=%.4f\n', ...
                max_iter, norm(V_b(1:3)), norm(V_b(4:6)));
    end
end
