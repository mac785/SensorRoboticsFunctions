function R = quat2rotm_my(q)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    % Input validation
    if ~(isnumeric(q) && isreal(q))
        error('quat2rotm_my:InvalidInputType', 'q must be a real numeric array.');
    end
    if size(q,2) ~= 4
        error('quat2rotm_my:InvalidInputSize', ...
              'q must be 1x4 or Nx4 with rows [w x y z].');
    end

    N = size(q,1);
    R = zeros(3,3,N);

    for i = 1:N
        qi = q(i,:);

        n = norm(qi);
        if n < eps
            error('quat2rotm_my:ZeroQuaternion', ...
                  'Quaternion norm is zero (row %d).', i);
        end
        qi = qi / n;

        w = qi(1); x = qi(2); y = qi(3); z = qi(4);

        % This is equivalent to MATLAB's convention.
        R(:,:,i) = [ 1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w);
                     2*(x*y + z*w),      1 - 2*(x*x + z*z), 2*(y*z - x*w);
                     2*(x*z - y*w),      2*(y*z + x*w),     1 - 2*(x*x + y*y) ];
    end

    if N == 1
        R = R(:,:,1);
    end

end