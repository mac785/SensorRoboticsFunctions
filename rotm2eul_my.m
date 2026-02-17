function eul = rotm2eul_my(R, sequence)
% ROTM2EUL_MY Convert rotation matrix/matrices to Euler angles.
%   eul = rotm2eul_my(R, sequence)
%
%   Inputs:
%     R:        3x3 or 3x3xN rotation matrix/matrices
%     sequence: 'ZYX' or 'ZYZ'  (matches MATLAB rotm2eul conventions)
%
%   Output:
%     eul: 1x3 (single) or Nx3 array of Euler angles [a b c] in radians
%          For 'ZYX': [z y x]
%          For 'ZYZ': [z y z]
%
%   Notes:
%     - Handles singularities (gimbal lock) for each convention.
%     - Returns row vectors like MATLAB.
%
%   Example:
%     e1 = rotm2eul_my(R,'ZYX');
%     e2 = rotm2eul_my(R,'ZYZ');

    % ZYX vs ZYZ decision
    if nargin < 2
        sequence = 'ZYX'; % default
    end
    sequence = upper(string(sequence));

    % input validation
    if ndims(R) == 2 %#ok<*ISMAT>
        if ~isequal(size(R), [3 3])
            error('rotm2eul_my:InvalidInputSize', ...
                  'R must be 3x3 or 3x3xN.');
        end
        R = reshape(R, 3, 3, 1);
    elseif ndims(R) == 3
        if size(R,1) ~= 3 || size(R,2) ~= 3
            error('rotm2eul_my:InvalidInputSize', ...
                  'R must be 3x3xN.');
        end
    else
        error('rotm2eul_my:InvalidInputSize', ...
              'R must be 3x3 or 3x3xN.');
    end

    N = size(R,3);
    eul = zeros(N,3);

    % singularity detection tolerance
    tol = 1e-6;

    for i = 1:N
        Ri = R(:,:,i);

        switch sequence
            case "ZYX"
                % MATLAB: eul = [z y x] = [yaw pitch roll]
                sy = -Ri(3,1);
                sy = max(-1, min(1, sy));
                y = asin(sy);

                if abs(abs(sy) - 1) < tol
                    % Gimbal lock at y = ±pi/2: set x = 0 and solve z
                    x = 0;
                    if sy < 0  % y = +pi/2
                        z = atan2(-Ri(1,2), Ri(2,2));
                    else       % y = -pi/2
                        z = atan2( Ri(1,2), Ri(2,2));
                    end
                else
                    z = atan2(Ri(2,1), Ri(1,1));
                    x = atan2(Ri(3,2), Ri(3,3));
                end

                eul(i,:) = [z y x];

            case "ZYZ"
                % MATLAB: eul = [z y z] for proper Euler angles.
                % Using convention: R = Rz(z1) * Ry(y) * Rz(z2)
                %
                % For general case, y = acos(R33)
                c = max(-1, min(1, Ri(3,3)));
                y = acos(c);

                % singularity detection
                if abs(sin(y)) < tol
                    z2 = 0;

                    z1 = atan2(Ri(2,1), Ri(1,1));

                else
                    % General case
                    z1 = atan2(Ri(2,3), Ri(1,3));
                    z2 = atan2(Ri(3,2), -Ri(3,1));
                end

                eul(i,:) = [z1 y z2];

            otherwise
                error('rotm2eul_my:UnsupportedSequence', ...
                      "Supported sequences are 'ZYX' and 'ZYZ'.");
        end
    end

    % Match MATLAB output shape for single input
    if N == 1
        eul = eul(1,:);
    end

end