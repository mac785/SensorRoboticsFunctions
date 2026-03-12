function axang = rotm2axang_my(R)
% ROTM2AXANG_MY Convert rotation matrix/matrices to axis-angle.
%   axang = rotm2axang_my(R)
%
%   Input:
%     R: 3x3 or 3x3xN rotation matrix/matrices
%
%   Output:
%     axang: 1x4 (single) or Nx4 array [ax ay az angle]
%
%   Notes:
%     - Matches MATLAB rotm2axang behavior.
%     - Axis is unit length.
%     - Angle is in [0, pi].

    % input validation
    if ndims(R) == 2 %#ok<*ISMAT>
        if ~isequal(size(R), [3 3])
            error('rotm2axang_my:InvalidInputSize', ...
                  'R must be 3x3 or 3x3xN.');
        end
        R = reshape(R, 3, 3, 1);
    elseif ndims(R) == 3
        if size(R,1) ~= 3 || size(R,2) ~= 3
            error('rotm2axang_my:InvalidInputSize', ...
                  'R must be 3x3xN.');
        end
    else
        error('rotm2axang_my:InvalidInputSize', ...
              'R must be 3x3 or 3x3xN.');
    end

    N = size(R,3);
    axang = zeros(N,4);

    for i = 1:N
        Ri = R(:,:,i);

        % --- Compute angle from trace ---
        cos_theta = (trace(Ri) - 1) / 2;

        % clamp to [-1, 1] to avoid NaNs
        cos_theta = max(-1, min(1, cos_theta));

        angle = acos(cos_theta);

        % handle small-angle case (≈ identity)
        if abs(angle) < 1e-12
            axis = [1 0 0];  % arbitrary
            angle = 0;

        % handle angle ≈ pi (most delicate case)
        elseif abs(pi - angle) < 1e-6
            % Extract axis from diagonal
            A = (Ri + eye(3)) / 2;
            axis = [sqrt(max(A(1,1),0)), ...
                    sqrt(max(A(2,2),0)), ...
                    sqrt(max(A(3,3),0))];

            % Fix signs using off-diagonal terms
            if Ri(1,2) < 0, axis(2) = -axis(2); end
            if Ri(1,3) < 0, axis(3) = -axis(3); end

            % Normalize to guard numerical issues
            axis = axis / norm(axis);

        else
            % CHANGE 5: general case
            axis = [Ri(3,2) - Ri(2,3), ...
                    Ri(1,3) - Ri(3,1), ...
                    Ri(2,1) - Ri(1,2)] / (2*sin(angle));

            axis = axis / norm(axis);
        end

        axang(i,:) = [axis angle];
    end

    % MATLAB output shape for single input
    if N == 1
        axang = axang(1,:);
    end
end