function [vol_lin, vol_ang] = J_ellipsoid_volume(robot, thetalist)
% J_ellipsoid_volume: Computes the volumes of the linear and angular
%                     velocity manipulability ellipsoids.
%
% The manipulability ellipsoid for linear (angular) velocities has semi-axis
% lengths equal to the singular values of the linear (angular) rows of the
% Jacobian.  The volume of a 3D ellipsoid with semi-axes a, b, c is:
%
%   V = (4/3) * pi * a * b * c
%
% In Lynch & Park convention, the body/space twist is V = [omega; v], so:
%   Angular part: rows 1-3 of Js  ->  Jw
%   Linear  part: rows 4-6 of Js  ->  Jv
%
% Inputs:
%   robot     - robot struct from KR210_params()
%   thetalist - n x 1 joint angle vector (radians)
%
% Outputs:
%   vol_lin - volume of the linear velocity ellipsoid  (m^3 / (rad/s)^n)
%   vol_ang - volume of the angular velocity ellipsoid (rad^3/s / (rad/s)^n)

    Js = J_space(robot.Slist, thetalist);

    % Separate angular (rows 1-3) and linear (rows 4-6) sub-Jacobians
    Jw = Js(1:3, :);
    Jv = Js(4:6, :);

    % Singular values of each sub-Jacobian give ellipsoid semi-axes
    sigma_w = svd(Jw);   % 3 values for a 6-DOF robot
    sigma_v = svd(Jv);

    vol_ang = (4/3) * pi * prod(sigma_w(1:3));
    vol_lin = (4/3) * pi * prod(sigma_v(1:3));

    fprintf('Ellipsoid volume  (angular): %.6f\n', vol_ang);
    fprintf('Ellipsoid volume  (linear) : %.6f\n', vol_lin);
end
