function T = FK_space(M, Slist, thetalist)
% FK_space: Computes forward kinematics using space-form PoE
% Inputs:
%   M         - 4x4 home configuration of end-effector
%   Slist     - 6xn matrix of space-form screw axes (columns)
%   thetalist - nx1 vector of joint angles (radians)
% Output:
%   T         - 4x4 end-effector transformation matrix

    T = eye(4);
    n = length(thetalist);
    for i = 1:n
        S_bracket = vecToSE3(Slist(:,i) * thetalist(i));
        T = T * MatrixExp6(S_bracket);
    end
    T = T * M;
end