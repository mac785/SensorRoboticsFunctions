function T = cloud2cloud(static_cloud,moving_cloud)
    %This function takes in data from a moving cloud and from a static
    %   cloud and finds the transformation between the two for a rigid object
    %   assuming the correspondence is known

    %This function uses the SVD method to calculate R

    % --------------- INPUTS -------------------- %
    %   static_cloud = [a_(x,1) a_(y,1) a_(z,1);
    %                   a_(x,2) a_(y,2) a_(z,2);
    %                   ...
    %                   a_(x,N) a_(y,N) a_(z,N)]
    %   moving_cloud = [b_(x,1) b_(y,1) b_(z,1);
    %                   b_(x,2) b_(y,2) b_(z,2);
    %                   ...
    %                   b_(x,N) b_(y,N) b_(z,N)]
    %
    %   N = number of points on rigid object
    % ------------------------------------------- %

    % --------------- OUTPUTS -------------------- %
    %   T = [R P;0 1] - transformation matrix between the clouds
    % -------------------------------------------- %
    
    A = static_cloud;
    B = moving_cloud;

    N = size(A,1); %number of points for cloud set
    if N < 3
        error('At least 3 points required');
    end
    %find centroid of each cloud set
    a_bar = mean(A,1);
    b_bar = mean(B,1);
    %find the distance between cloud centroid and measured points
    a_tilda_matrix = A - a_bar;
    b_tilda_matrix = B - b_bar;

    %------------ Solve for R -------------------
    
    %determine H
    H = a_tilda_matrix'*b_tilda_matrix;
    %use SVD to determine U,S,V
    [U,~,V] = svd(H);
    X = V*U';
    
    %check determinant of X
    tol = 1e-8;
    if abs(det(X) - 1) < tol  % proper rotation
        R = X;
    elseif abs(det(X) + 1) < tol  % reflection — apply standard fix
        % Occurs when markers are nearly coplanar (smallest singular value
        % of H is small). For rigid body data this is always the correct fix.
        V_prime = V;
        V_prime(:,3) = -V(:,3);
        R = V_prime*U';
    else
        T = NaN(4,4);
        warning('cloud2cloud: det(X) = %.6f, not a valid rotation or reflection.', det(X));
        return
    end

    %------------ Solve for P -------------------
    P = b_bar' - R*a_bar'; %now a column vector

    %------------ Solve for T -------------------
    T = [R P; zeros(1,3) 1];

end