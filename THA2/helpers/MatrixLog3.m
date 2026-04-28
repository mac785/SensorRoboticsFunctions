function so3mat = MatrixLog3(R)
% MatrixLog3: Matrix logarithm of a rotation matrix R in SO(3).
%
% Returns the 3x3 skew-symmetric matrix [omega]*theta such that
% MatrixExp3([omega]*theta) = R.
%
% Input:
%   R      - 3x3 rotation matrix
%
% Output:
%   so3mat - 3x3 skew-symmetric matrix (element of so(3))
%
% Reference: Lynch & Park, Modern Robotics, Algorithm 3.1

    acosinput = (trace(R) - 1) / 2;

    if acosinput >= 1           % R = I, theta = 0
        so3mat = zeros(3, 3);

    elseif acosinput <= -1      % theta = pi special case
        % Find omega from (R + I)/2 = omega*omega'
        % Choose the column with the largest diagonal entry for numerical stability
        if abs(1 + R(3,3)) > 1e-10
            omg = (1 / sqrt(2*(1 + R(3,3)))) * [R(1,3); R(2,3); 1 + R(3,3)];
        elseif abs(1 + R(2,2)) > 1e-10
            omg = (1 / sqrt(2*(1 + R(2,2)))) * [R(1,2); 1 + R(2,2); R(3,2)];
        else
            omg = (1 / sqrt(2*(1 + R(1,1)))) * [1 + R(1,1); R(2,1); R(3,1)];
        end
        so3mat = vecToSO3(pi * omg);

    else                        % general case
        theta  = acos(acosinput);
        so3mat = (theta / (2 * sin(theta))) * (R - R');
    end
end
