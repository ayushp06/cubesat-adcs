function params = mekfParams()
% MEKFPARAMS Primary attitude-estimator tuning in SI units.
    params.gyroNoiseStd = deg2rad(0.02);
    params.biasRandomWalkStd = deg2rad(0.002);
    params.magnetometerVectorNoiseStd = 0.008;
    params.sunVectorNoiseStd = 0.015;
    params.initialAttitudeStd = deg2rad(15);
    params.initialBiasStd = deg2rad(0.5);
end
