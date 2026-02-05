function [axis, angle] = rotationMatrixToAxisAngle(R)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    if ~isequal(size(R), [3 3])
        error('Invalid matrix size, must be 3x3');
    end

    I = eye(3);
    tolerance = 1e-6;

    if norm(R-I, 'fro') < tolerance % frobenius norm, found online no idea if this is the right one
        axis = [1;0;0]; % arbitrary, actually undefined
        angle = 0;
        return;
    end

    angle = acos((trace(R) - 1) /2);
    
    axis = 1/(2*sin(angle)) * ...
        [ R(3,2) - R(2,3);
          R(1,3) - R(3,1);
          R(2,1) - R(1,2) ];
end