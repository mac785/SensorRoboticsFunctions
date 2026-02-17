function q = rotm2quat_my(R)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    % Input validation
    if ndims(R) == 2 %#ok<*ISMAT>
        if ~isequal(size(R), [3 3])
            error('rotm2quat_my:InvalidInputSize', ...
                  'R must be 3x3 or 3x3xN.');
        end
        R = reshape(R, 3, 3, 1);
    elseif ndims(R) == 3
        if size(R,1) ~= 3 || size(R,2) ~= 3
            error('rotm2quat_my:InvalidInputSize', ...
                  'R must be 3x3xN.');
        end
    else
        error('rotm2quat_my:InvalidInputSize', ...
              'R must be 3x3 or 3x3xN.');
    end

    N = size(R,3);
    q = zeros(N,4);

    % Trace batching
    for i = 1:N
        Ri = R(:,:,i);
        tr = trace(Ri);

        if tr > 0
            S = sqrt(tr + 1.0) * 2; % S = 4*w
            w = 0.25 * S;
            x = (Ri(3,2) - Ri(2,3)) / S;
            y = (Ri(1,3) - Ri(3,1)) / S;
            z = (Ri(2,1) - Ri(1,2)) / S;

        else
            % Find largest diagonal element for numerical stability
            if Ri(1,1) > Ri(2,2) && Ri(1,1) > Ri(3,3)
                S = sqrt(1.0 + Ri(1,1) - Ri(2,2) - Ri(3,3)) * 2;
                w = (Ri(3,2) - Ri(2,3)) / S;
                x = 0.25 * S;
                y = (Ri(1,2) + Ri(2,1)) / S;
                z = (Ri(1,3) + Ri(3,1)) / S;

            elseif Ri(2,2) > Ri(3,3)
                S = sqrt(1.0 + Ri(2,2) - Ri(1,1) - Ri(3,3)) * 2;
                w = (Ri(1,3) - Ri(3,1)) / S;
                x = (Ri(1,2) + Ri(2,1)) / S;
                y = 0.25 * S;
                z = (Ri(2,3) + Ri(3,2)) / S;

            else
                S = sqrt(1.0 + Ri(3,3) - Ri(1,1) - Ri(2,2)) * 2;
                w = (Ri(2,1) - Ri(1,2)) / S;
                x = (Ri(1,3) + Ri(3,1)) / S;
                y = (Ri(2,3) + Ri(3,2)) / S;
                z = 0.25 * S;
            end
        end

        qi = [w x y z];

        qi = qi / norm(qi);

        % enforce MATLAB-style sign convention (w >= 0)
        if qi(1) < 0
            qi = -qi;
        end

        q(i,:) = qi;
    end

    % MATLAB output shape for single input
    if N == 1
        q = q(1,:);
    end
end