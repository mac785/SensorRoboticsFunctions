function eul = rotm2eul_my(R, sequence)
% ROTM2EUL_MY Convert rotation matrix/matrices to Euler angles.
%   eul = rotm2eul_my(R, sequence)
%
%   Inputs:
%     R:        3x3 or 3x3xN rotation matrix/matrices
%     sequence: 'ZYX' or 'ZYZ'
%
%   Output:
%     eul: 1x3 or Nx3 array of Euler angles
%          ZYX → [z y x]
%          ZYZ → [z y z]

    if nargin < 2
        sequence = 'ZYX';
    end
    sequence = upper(string(sequence));

    % --- validate R ---
    if ndims(R) == 2 %#ok<*ISMAT>
        if ~isequal(size(R), [3 3])
            error('rotm2eul_my:InvalidInputSize','R must be 3x3 or 3x3xN.');
        end
        R = reshape(R,3,3,1);
    elseif ndims(R) == 3
        if size(R,1)~=3 || size(R,2)~=3
            error('rotm2eul_my:InvalidInputSize','R must be 3x3xN.');
        end
    else
        error('rotm2eul_my:InvalidInputSize','R must be 3x3 or 3x3xN.');
    end

    N = size(R,3);
    eul = zeros(N,3);
    tol = 1e-6;

    for i = 1:N
        Ri = R(:,:,i);

        switch sequence

        % ================= ZYX =================
        case "ZYX"
            sy = -Ri(3,1);
            sy = max(-1, min(1, sy));
            y = asin(sy);

            if abs(abs(sy) - 1) < tol
                % Gimbal lock
                x = 0;
                if sy < 0
                    z = atan2(-Ri(1,2), Ri(2,2));
                else
                    z = atan2( Ri(1,2), Ri(2,2));
                end
            else
                z = atan2(Ri(2,1), Ri(1,1));
                x = atan2(Ri(3,2), Ri(3,3));
            end

            eul(i,:) = [z y x];

        % ================= ZYZ =================
        case "ZYZ"
            c = max(-1, min(1, Ri(3,3)));
            y = acos(c);

            if abs(sin(y)) < tol
                % Singularity: collapse Z angles
                z2 = 0;
                z1 = atan2(Ri(2,1), Ri(1,1));
            else
                z1 = atan2(Ri(2,3), Ri(1,3));
                z2 = atan2(Ri(3,2), -Ri(3,1));
            end

            eul(i,:) = [z1 y z2];

        otherwise
            error('rotm2eul_my:UnsupportedSequence', ...
                  "Supported sequences: 'ZYX', 'ZYZ'");
        end
    end

    % match MATLAB shape
    if N == 1
        eul = eul(1,:);
    end
end