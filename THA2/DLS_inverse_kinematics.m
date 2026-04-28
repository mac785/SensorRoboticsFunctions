function [thetalist, success, iter, history] = DLS_inverse_kinematics(robot, T_desired, thetalist0, ...
                                                               lambda_max, sigma_thresh, ...
                                                               eomg, ev, max_iter, verbose)
% DLS_inverse_kinematics: Damped Least Squares (DLS) iterative inverse kinematics.
%
% Near singularities the standard pseudoinverse J^+ = J^T(JJ^T)^{-1} amplifies
% errors because small singular values appear in the denominator.  DLS replaces
% the pseudoinverse with:
%
%   J_dls = J^T (J J^T + lambda^2 I)^{-1}
%
% which bounds the effective gain even when singular values approach zero.
%
% Variable damping (Nakamura & Hanafusa, 1986):
%   - When sigma_min >= sigma_thresh:  lambda = 0  (reduces to standard NR)
%   - When sigma_min <  sigma_thresh:  lambda^2 = lambda_max^2 *
%                                          (1 - (sigma_min/sigma_thresh)^2)
%   so damping turns on smoothly as the robot approaches a singularity.
%
% Update rule:
%   V_b     = vec( log( T_curr^{-1} * T_desired ) )
%   theta  += J_b^T (J_b J_b^T + lambda^2 I)^{-1} * V_b
%
% Inputs:
%   robot        - robot struct from KR120_params()
%   T_desired    - 4x4 desired end-effector transform
%   thetalist0   - n x 1 initial joint angle guess (radians)
%   lambda_max   - maximum damping factor  (default: 0.1)
%   sigma_thresh - minimum singular value below which damping activates
%                  (default: 0.05)
%   eomg         - angular error tolerance (rad)  (default: 1e-3)
%   ev           - linear  error tolerance (m)    (default: 1e-3)
%   max_iter     - maximum iterations             (default: 200)
%   verbose      - print convergence result       (default: true)
%
% Outputs:
%   thetalist - n x 1 solution joint angles (radians)
%   success   - true if converged within tolerances
%   iter      - number of iterations taken
%   history   - (optional) struct array of per-iteration state; requesting
%               this 4th output enables recording (see record_ik_state.m)

    if nargin < 4, lambda_max   = 0.1;  end
    if nargin < 5, sigma_thresh = 0.05; end
    if nargin < 6, eomg         = 1e-3; end
    if nargin < 7, ev           = 1e-3; end
    if nargin < 8, max_iter     = 200;  end
    if nargin < 9, verbose      = true; end

    do_record = nargout >= 4;
    history   = [];
    if do_record
        history = struct('theta',{},'T_curr',{},'omg_err',{},'lin_err',{}, ...
                         'kappa',{},'iso',{},'vol_lin',{},'vol_ang',{}, ...
                         'lambda',{},'converged',{},'elapsed_s',{});
        t_start = tic;
    end

    thetalist = thetalist0(:);
    m         = 6;
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

        %% Body Jacobian and variable damping factor
        Jb        = J_body(robot.Blist, thetalist);
        sigma_min = min(svd(Jb));
        if sigma_min >= sigma_thresh
            lambda2 = 0;
        else
            lambda2 = lambda_max^2 * (1 - (sigma_min / sigma_thresh)^2);
        end

        %% Write actual lambda into this iteration's history entry
        if do_record, history(end).lambda = sqrt(lambda2); end

        %% DLS update: J^T (J J^T + lambda^2 I)^{-1} V_b
        thetalist = thetalist + Jb' * ((Jb * Jb' + lambda2 * eye(m)) \ V_b);
    end

    if verbose
        if success
            fprintf('DLS IK converged in %d iterations.\n', iter);
        else
            fprintf('DLS IK did NOT converge after %d iterations. Final error: omg=%.4f v=%.4f\n', ...
                    max_iter, norm(V_b(1:3)), norm(V_b(4:6)));
        end
    end
end
