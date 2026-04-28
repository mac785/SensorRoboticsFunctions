function q = rotationMatrixToQuaternion(R)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    if ~isequal(size(R), [3 3])
        error('Invalid matrix size, must be 3x3');
    end

    q0 = (1/2)*sqrt(R(1,1) + R(2,2) + R(3,3) + 1);
    q1 = (1/2)*sign(R(3,2) - R(2,3)) * sqrt(R(1,1) - R(2,2) - R(3,3) + 1);
    q2 = (1/2)*sign(R(1,3) - R(3,1)) * sqrt(R(2,2) - R(3,3) - R(1,1) + 1);
    q3 = (1/2)*sign(R(2,1) - R(1,2)) * sqrt(R(3,3) - R(1,1) - R(2,2) + 1);
    q = [q0; q1; q2; q3];
end