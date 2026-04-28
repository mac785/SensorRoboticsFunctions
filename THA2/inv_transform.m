function Tinv = inv_transform(T)
% Analytically inverts a homogeneous transform
    R = T(1:3,1:3);
    p = T(1:3,4);
    Tinv = [R', -R'*p; 0,0,0,1];
end