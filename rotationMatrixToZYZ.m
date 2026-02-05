function eul = rotationMatrixToZYZ(R)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

    if ~isequal(size(R), [3 3])
        error('Invalid matrix size, must be 3x3');
    end

    phi = atan2(R(2,3),R(1,3));

    theta = atan2(sqrt(R(1,3)^2 + R(2,3)^2),R(3,3));

    psi = atan2(R(3,2), -R(3,1));

    if theta > pi/2 || theta < 0
        phi = atan2(-R(2,3),-R(1,3));

        theta = atan2(-sqrt(R(1,3)^2 + R(2,3)^2),R(3,3));

        psi = atan2(-R(3,2), R(3,1));

    end

    eul = [phi; theta; psi];
end