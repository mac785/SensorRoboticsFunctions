function R = MatrixExp3(so3mat)
    % so3mat is a 3x3 skew-symmetric matrix
    omgtheta = SO3ToVec(so3mat);
    theta = norm(omgtheta);
    if theta < 1e-10
        R = eye(3);
    else
        omghat = so3mat / theta;
        R = eye(3) + sin(theta)*omghat + (1-cos(theta))*(omghat^2);
    end
end