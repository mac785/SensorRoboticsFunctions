function Jb = J_body(Blist, thetalist)
% J_body: Computes the body-form Jacobian
% Inputs:
%   Blist     - 6xn matrix of body-form screw axes (columns)
%   thetalist - nx1 vector of joint angles (radians)
% Output:
%   Jb        - 6xn body Jacobian matrix

    n = length(thetalist);
    Jb = zeros(6, n);
    T = eye(4);
    
    for i = n:-1:1
        % Last column is just B_n, then work backwards
        Jb(:,i) = adjoint_map(T) * Blist(:,i);
        T = T * MatrixExp6(vecToSE3(-Blist(:,i) * thetalist(i)));
    end
end