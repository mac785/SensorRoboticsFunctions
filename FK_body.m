function T = FK_body(M, Blist, thetalist)
% FK_body: Computes forward kinematics using body-form PoE
% Inputs:
%   M         - 4x4 home configuration of end-effector
%   Blist     - 6xn matrix of body-form screw axes (columns)
%   thetalist - nx1 vector of joint angles (radians)
% Output:
%   T         - 4x4 end-effector transformation matrix

    T = M;
    n = length(thetalist);
    for i = 1:n
        B_bracket = vecToSE3(Blist(:,i) * thetalist(i));
        T = T * MatrixExp6(B_bracket);
    end
end