function eul = rotationMatrixToZYX(R)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    if ~isequal(size(R), [3 3])
        error('Invalid matrix size, must be 3x3');
    end

    phi = atan2(R(2,1), R(1,1));

    theta = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));

    psi = atan2(R(3,2), R(3,3));

    if theta < -pi/2 || theta > pi/2

        phi = atan2(-R(2,1), -R(1,1));

        theta = atan2(-R(3,1),-sqrt(R(3,2)^2 + R(3,3)^2));

        psi = atan2(-R(3,2), -R(3,3));

    end

    eul = [phi; theta; psi];
end