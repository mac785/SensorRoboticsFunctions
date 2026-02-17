function R = axang2rotm_my(axang)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    % Input validation
    axis = axis(:);
    if numel(axis) ~= 3
        error('axisAngleToRotationMatrix:InvalidAxis', ...
              'axis must be a 3-element vector.');
    end
    if ~isscalar(angle)
        error('axisAngleToRotationMatrix:InvalidAngle', ...
              'angle must be a scalar.');
    end

    % normalize axis
    na = norm(axis);
    if na < eps
        if abs(angle) < 1e-12
            R = eye(3);
            return;
        else
            error('axisAngleToRotationMatrix:ZeroAxis', ...
                  'axis is zero-length but angle is nonzero.');
        end
    end

    u = axis / na;

    % Tiny angles check - helps with floating point error
    if abs(angle) < 1e-12
        R = eye(3);
        return;
    end

    % building skew matrix from normalized axis
    axis_skew = [ 0 -u(3) u(2);
                u(3) 0 -u(1);
                -u(2) u(1) 0 ];
    
    % Rodrigues'  formula
    R = eye(3) + sin(angle)*axis_skew + (1 - cos(angle))*(axis_skew*axis_skew);

end