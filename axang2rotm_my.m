function R = axang2rotm_my(axang)
% AXANG2ROTM_MY Convert axis-angle to rotation matrix.
%   R = axang2rotm_my(axang)
%   axang: 1x4 or Nx4 array, each row [ax ay az angle] (radians)
%   R: 3x3 (if 1x4) or 3x3xN (if Nx4)

    if size(axang, 2) ~= 4
        error('axang2rotm_my:InvalidInput', ...
              'axang must be Nx4 with rows [ax ay az angle].');
    end

    N = size(axang,1);
    R = zeros(3,3,N);

    for i = 1:N
        axis  = axang(i,1:3).';
        angle = axang(i,4);

        if ~isscalar(angle) || ~isfinite(angle)
            error('axang2rotm_my:InvalidAngle', ...
                  'Angle must be a finite scalar.');
        end

        % Normalize axis
        na = norm(axis);
        if na < eps
            if abs(angle) < 1e-12
                R(:,:,i) = eye(3);
                continue;
            else
                error('axang2rotm_my:ZeroAxis', ...
                      'Axis is zero-length but angle is nonzero (row %d).', i);
            end
        end
        u = axis / na;

        % Tiny angles -> identity (numerical stability)
        if abs(angle) < 1e-12
            R(:,:,i) = eye(3);
            continue;
        end

        % Skew-symmetric matrix from unit axis
        axis_skew = [  0   -u(3)  u(2);
                      u(3)  0    -u(1);
                     -u(2)  u(1)  0  ];

        % Rodrigues' formula
        R(:,:,i) = eye(3) + sin(angle)*axis_skew + (1 - cos(angle))*(axis_skew*axis_skew);
    end

    % Match MATLAB behavior: if N==1, return 3x3 not 3x3x1
    if N == 1
        R = R(:,:,1);
    end

end