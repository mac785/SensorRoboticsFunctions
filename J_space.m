function Js = J_space(Slist, thetalist)
% J_space: Computes the space-form Jacobian
% Inputs:
%   Slist     - 6xn matrix of space-form screw axes (columns)
%   thetalist - nx1 vector of joint angles (radians)
% Output:
%   Js        - 6xn space Jacobian matrix

    n = length(thetalist);
    Js = zeros(6, n);
    T = eye(4);
    
    for i = 1:n
        % First column is just S1, then each subsequent column is
        % transformed by the accumulated partial FK up to joint i-1
        Js(:,i) = adjoint_map(T) * Slist(:,i);
        T = T * MatrixExp6(vecToSE3(Slist(:,i) * thetalist(i)));
    end
end