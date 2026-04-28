function T = MatrixExp6(se3mat)
    omgtheta = SO3ToVec(se3mat(1:3,1:3));
    theta = norm(omgtheta);
    if theta < 1e-10
        T = [eye(3), se3mat(1:3,4); 0,0,0,1];
    else
        omghat = se3mat(1:3,1:3) / theta;
        G = eye(3)*theta + (1-cos(theta))*omghat + ...
            (theta - sin(theta))*(omghat^2);
        T = [MatrixExp3(se3mat(1:3,1:3)), G*(se3mat(1:3,4)/theta);
             0, 0, 0, 1];
    end
end