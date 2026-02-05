function R = axisAngleToRotationMatrix(axis, angle)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    axis_skew = [ 0 -axis(3) axis(2);
           axis(3) 0 -axis(1);
           -axis(2) axis(1) 0 ];

    R = eye(3) + sin(angle)*axis_skew + (1 - cos(angle))*(axis_skew*axis_skew);

end