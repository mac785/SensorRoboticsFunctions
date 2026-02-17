function R = eul2rotm_my(eul, sequence)
% EUL2ROTM_MY Convert Euler angles to rotation matrix/matrices.
%   R = eul2rotm_my(eul, sequence)
%
%   Inputs:
%     eul:      1x3 or Nx3 array of Euler angles (radians)
%     sequence: 'ZYX' or 'ZYZ' (matches MATLAB eul2rotm conventions)
%
%   Output:
%     R: 3x3 (if eul is 1x3) or 3x3xN (if eul is Nx3)
%
%   Conventions:
%     - 'ZYX': eul = [z y x] and R = Rz(z)*Ry(y)*Rx(x)
%     - 'ZYZ': eul = [z1 y z2] and R = Rz(z1)*Ry(y)*Rz(z2)

    if nargin < 2
        sequence = 'ZYX';
    end
    sequence = upper(string(sequence));

    if ~(isnumeric(eul) && isreal(eul))
        error('eul2rotm_my:InvalidInputType', 'eul must be a real numeric array.');
    end
    if size(eul,2) ~= 3
        error('eul2rotm_my:InvalidInputSize', ...
              'eul must be 1x3 or Nx3.');
    end

    N = size(eul,1);
    R = zeros(3,3,N);

    for i = 1:N
        a = eul(i,1);
        b = eul(i,2);
        c = eul(i,3);

        switch sequence
            case "ZYX"
                % eul = [z y x]
                cz = cos(a); sz = sin(a);
                cy = cos(b); sy = sin(b);
                cx = cos(c); sx = sin(c);

                % R = Rz(z)*Ry(y)*Rx(x)
                R(:,:,i) = [ cz*cy,  cz*sy*sx - sz*cx,  cz*sy*cx + sz*sx;
                             sz*cy,  sz*sy*sx + cz*cx,  sz*sy*cx - cz*sx;
                             -sy,     cy*sx,             cy*cx ];

            case "ZYZ"
                % eul = [z1 y z2]
                cz1 = cos(a); sz1 = sin(a);
                cy  = cos(b); sy  = sin(b);
                cz2 = cos(c); sz2 = sin(c);

                % R = Rz(z1)*Ry(y)*Rz(z2)
                R(:,:,i) = [ cz1*cy*cz2 - sz1*sz2,  -cz1*cy*sz2 - sz1*cz2,  cz1*sy;
                             sz1*cy*cz2 + cz1*sz2,  -sz1*cy*sz2 + cz1*cz2,  sz1*sy;
                             -sy*cz2,               sy*sz2,                cy ];

            otherwise
                error('eul2rotm_my:UnsupportedSequence', ...
                      "Supported sequences are 'ZYX' and 'ZYZ'.");
        end
    end

    % Match MATLAB output shape for single input
    if N == 1
        R = R(:,:,1);
    end
end
