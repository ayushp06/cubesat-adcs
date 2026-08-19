function estimate = mekfUpdateVectors(estimate, measuredBody, referenceECI, noiseStd)
% MEKFUPDATEVECTORS Correct attitude/bias from one or more direction vectors.
    count = size(measuredBody,2); innovation = zeros(3*count,1); H = zeros(3*count,6);
    for index = 1:count
        measured = measuredBody(:,index)/norm(measuredBody(:,index));
        predicted = quatToDCM(estimate.qIB)' * ...
            (referenceECI(:,index)/norm(referenceECI(:,index)));
        rows = 3*index-2:3*index;
        innovation(rows) = measured - predicted;
        H(rows,1:3) = skew(predicted);
    end
    if isscalar(noiseStd), noiseStd = noiseStd*ones(1,count); end
    R = diag(kron(noiseStd.^2,ones(1,3)));
    S = H*estimate.covariance*H' + R;
    gain = (estimate.covariance*H')/S;
    correction = gain*innovation;
    deltaQuaternion = quatNormalize([1; correction(1:3)/2]);
    estimate.qIB = quatNormalize(quatMultiply(estimate.qIB,deltaQuaternion));
    estimate.gyroBias = estimate.gyroBias + correction(4:6);
    identity = eye(6);
    estimate.covariance = (identity-gain*H)*estimate.covariance*(identity-gain*H)' + gain*R*gain';
    estimate.covariance = (estimate.covariance+estimate.covariance')/2;
    estimate.lastInnovation = innovation;
    estimate.lastInnovationCovariance = S;
    estimate.lastNIS = innovation'*(S\innovation);
end
