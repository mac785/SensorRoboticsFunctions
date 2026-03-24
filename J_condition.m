function kappa = J_condition(robot, thetalist)
% J_condition: Computes the condition number of the robot Jacobian.
%
% The condition number quantifies how far a matrix is from being singular
% and how sensitive the IK solution is to small errors.  A value of 1
% is ideal (well-conditioned); large values indicate near-singularity.
%
%   kappa = sigma_max / sigma_min
%
% where sigma_i are the singular values of the full 6xn space Jacobian.
%
% Inputs:
%   robot     - robot struct from KR120_params()
%   thetalist - n x 1 joint angle vector (radians)
%
% Output:
%   kappa - condition number in [1, Inf)

    Js    = J_space(robot.Slist, thetalist);
    sigma = svd(Js);          % singular values, descending

    if sigma(end) < 1e-10
        kappa = Inf;
    else
        kappa = sigma(1) / sigma(end);
    end

    fprintf('Condition number: %.6f\n', kappa);
end
