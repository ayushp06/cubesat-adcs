function sensors = sensorParams()
% SENSORPARAMS Nominal flight-like sensor assumptions in SI units.
    sensors.gyro.samplePeriod = 0.01;
    sensors.gyro.noiseStd = deg2rad(0.02);
    sensors.gyro.biasRandomWalkStd = deg2rad(0.002);
    sensors.gyro.scaleFactor = [300; -200; 150] * 1e-6;
    sensors.gyro.misalignment = deg2rad([0, 0.03, -0.02; -0.01, 0, 0.02; 0.01, -0.02, 0]);
    sensors.gyro.maxRate = deg2rad(250);

    sensors.magnetometer.samplePeriod = 0.1;
    sensors.magnetometer.noiseStd = 150e-9;
    sensors.magnetometer.bias = [300; -200; 100] * 1e-9;
    sensors.magnetometer.scaleFactor = [500; -300; 200] * 1e-6;
    sensors.magnetometer.misalignment = deg2rad([0, 0.1, -0.05; -0.08, 0, 0.04; 0.03, -0.06, 0]);
    sensors.magnetometer.maxField = 100e-6;

    sensors.sun.samplePeriod = 0.2;
    sensors.sun.noiseStd = 0.01;
    sensors.sun.detectionThreshold = 0.05;

    sensors.gps.samplePeriod = 1;
    sensors.gps.positionNoiseStd = 3;
    sensors.gps.velocityNoiseStd = 0.05;
    sensors.gps.dropoutProbability = 0.01;

    sensors.starTracker.enabled = false;
    sensors.starTracker.samplePeriod = 1;
    sensors.starTracker.angleNoiseStd = deg2rad(0.01);
    sensors.starTracker.dropoutProbability = 0.02;
end
