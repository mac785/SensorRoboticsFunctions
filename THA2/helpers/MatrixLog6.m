function se3mat = MatrixLog6(T)
% MatrixLog6: Matrix logarithm of a homogeneous transform T in SE(3).
%
% Returns the 4x4 matrix [se3] = [[omega]*theta, v*theta; 0,0,0,0]
% such that MatrixExp6(se3mat) = T.
%
% Input:
%   T      - 4x4 homogeneous transformation matrix
%
% Output:
%   se3mat - 4x4 element of se(3)
%
% Reference: Lynch & Park, Modern Robotics, Algorithm 3.2

    R = T(1:3, 1:3);
    p = T(1:3, 4);

    omgmat = MatrixLog3(R);
    omgtheta = SO3ToVec(omgmat);    % omega*theta as a 3-vector
    theta = norm(omgtheta);

    if theta < 1e-10
        % Pure translation: no rotation
        se3mat = [zeros(3,3),  p;
                  0, 0, 0,     0];
    else
        % G^{-1}(theta) = I/theta - omghat/2 + (1/theta - cot(theta/2)/2)*omghat^2
        % (Lynch & Park Eq. 3.88)
        omghat = omgmat / theta;          % normalised skew matrix
        G_inv  = eye(3)/theta ...
                 - omghat/2 ...
                 + (1/theta - 0.5*(cos(theta/2)/sin(theta/2))) * (omghat^2);
        v = G_inv * p;                    % screw axis linear component
        se3mat = [omgmat,  v * theta;
                  0, 0, 0, 0];
    end
end
