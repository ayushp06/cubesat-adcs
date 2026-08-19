function q = dcmToQuat(dcm)
% DCMTOQUAT Convert a proper body-to-inertial DCM to scalar-first quaternion.
    K = [dcm(1,1)-dcm(2,2)-dcm(3,3), dcm(2,1)+dcm(1,2), dcm(3,1)+dcm(1,3), dcm(2,3)-dcm(3,2); ...
         dcm(2,1)+dcm(1,2), dcm(2,2)-dcm(1,1)-dcm(3,3), dcm(3,2)+dcm(2,3), dcm(3,1)-dcm(1,3); ...
         dcm(3,1)+dcm(1,3), dcm(3,2)+dcm(2,3), dcm(3,3)-dcm(1,1)-dcm(2,2), dcm(1,2)-dcm(2,1); ...
         dcm(2,3)-dcm(3,2), dcm(3,1)-dcm(1,3), dcm(1,2)-dcm(2,1), trace(dcm)] / 3;
    [vectors,values] = eig(K);
    [~,index] = max(diag(values));
    qVectorFirst = vectors(:,index);
    q = quatNormalize([qVectorFirst(4); -qVectorFirst(1:3)]);
    if q(1) < 0, q = -q; end
end
