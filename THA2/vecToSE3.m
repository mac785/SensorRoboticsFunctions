function T = vecToSE3(V)
    % V = [omega; v], 6x1 twist
    T = [vecToSO3(V(1:3)), V(4:6);
         0, 0, 0,           0     ];
end