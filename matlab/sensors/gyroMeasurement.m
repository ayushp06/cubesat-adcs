function [measurement, bias] = gyroMeasurement(omegaB, bias, dt, params)
% GYROMEASUREMENT Sample rate gyro with bias walk, calibration errors, and saturation.
    bias = bias + params.biasRandomWalkStd * sqrt(dt) * randn(3,1);
    measurement = (eye(3) + diag(params.scaleFactor)) * ...
        (eye(3) + params.misalignment) * omegaB + bias + params.noiseStd * randn(3,1);
    measurement = max(-params.maxRate, min(params.maxRate, measurement));
end
