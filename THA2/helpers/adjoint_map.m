function AdT = adjoint_map(T)
% 6x6 adjoint representation of transform T
    R = T(1:3,1:3);
    p = T(1:3,4);
    AdT = [R,           zeros(3);
           vecToSO3(p)*R,  R    ];
end