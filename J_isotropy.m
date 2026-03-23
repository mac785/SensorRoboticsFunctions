function iso = J_isotropy(robot, thetalist)
% J_isotropy: Computes the isotropy index of the robot Jacobian.
%
% The isotropy index measures how "sphere-like" the manipulability ellipsoid
% is.  A value of 1 means the robot can move equally in all task-space
% directions (isotropic); 0 means the configuration is singular.
%
%   iso = sigma_min / sigma_max
%
% where sigma_i are the singular values of the full 6xn space Jacobian.
%
% Inputs:
%   robot     - robot struct from KR210_params()
%   thetalist - n x 1 joint angle vector (radians)
%
% Output:
%   iso - isotropy index in [0, 1]

    Js    = J_space(robot.Slist, thetalist);
    sigma = svd(Js);          % singular values, descending

    if sigma(1) < 1e-10
        iso = 0;
    else
        iso = sigma(end) / sigma(1);
    end

    fprintf('Isotropy index: %.6f\n', iso);
end
