function v = SO3ToVec(so3mat)
    v = [so3mat(3,2); so3mat(1,3); so3mat(2,1)];
end