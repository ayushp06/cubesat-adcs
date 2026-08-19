function qIB = questAttitude(bodyVectors, inertialVectors, weights)
% QUESTATTITUDE Davenport q-method solution of Wahba's problem.
    if nargin < 3, weights = ones(1,size(bodyVectors,2)); end
    profile = zeros(3);
    for index = 1:size(bodyVectors,2)
        body = bodyVectors(:,index)/norm(bodyVectors(:,index));
        inertial = inertialVectors(:,index)/norm(inertialVectors(:,index));
        profile = profile + weights(index) * body * inertial';
    end
    sigma = trace(profile); symmetric = profile + profile';
    z = [profile(2,3)-profile(3,2); profile(3,1)-profile(1,3); profile(1,2)-profile(2,1)];
    K = [symmetric-sigma*eye(3), z; z', sigma];
    [vectors,values] = eig(K);
    [~,index] = max(diag(values));
    qVectorFirst = vectors(:,index);
    qIB = quatNormalize([qVectorFirst(4); qVectorFirst(1:3)]);
    if qIB(1) < 0, qIB = -qIB; end
end
