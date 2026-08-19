function estimate = mekfPredict(estimate, gyroMeasurement, dt, params)
% MEKFPREDICT Propagate nominal attitude, gyro bias, and error covariance.
    omega = gyroMeasurement - estimate.gyroBias;
    angle = norm(omega) * dt;
    if angle == 0
        increment = [1;0;0;0];
    else
        increment = [cos(angle/2); sin(angle/2)*omega/norm(omega)];
    end
    estimate.qIB = quatNormalize(quatMultiply(estimate.qIB, increment));

    F = [-skew(omega), -eye(3); zeros(3), zeros(3)];
    transition = eye(6) + F*dt;
    processNoise = diag([params.gyroNoiseStd^2*dt*ones(1,3), ...
        params.biasRandomWalkStd^2*dt*ones(1,3)]);
    estimate.covariance = transition*estimate.covariance*transition' + processNoise;
    estimate.covariance = (estimate.covariance + estimate.covariance')/2;
end
