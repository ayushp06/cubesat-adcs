function estimate = mekfInitialize(qIB, gyroBias, params)
% MEKFINITIALIZE Nominal quaternion/bias and six-state error covariance.
    estimate.qIB = quatNormalize(qIB);
    estimate.gyroBias = gyroBias;
    estimate.covariance = diag([params.initialAttitudeStd^2*ones(1,3), ...
        params.initialBiasStd^2*ones(1,3)]);
    estimate.lastInnovation = [];
    estimate.lastInnovationCovariance = [];
    estimate.lastNIS = NaN;
end
