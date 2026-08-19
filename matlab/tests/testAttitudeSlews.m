clear;
clc;

testDir = fileparts(mfilename("fullpath"));
addpath(fullfile(testDir, "..", "config"));
addpath(fullfile(testDir, "..", "control"));
addpath(fullfile(testDir, "..", "dynamics"));
addpath(fullfile(testDir, "..", "math"));
addpath(fullfile(testDir, "..", "simulations"));

rw = reactionWheelParams();
halfAngle = pi / 4;
axis = [1; 2; 3] / norm([1; 2; 3]);
references = [ ...
    cos(halfAngle), cos(halfAngle), cos(halfAngle), cos(halfAngle), cos(halfAngle)
    sin(halfAngle), 0, 0, axis(1) * sin(halfAngle), axis(1) * sin(halfAngle)
    0, sin(halfAngle), 0, axis(2) * sin(halfAngle), axis(2) * sin(halfAngle)
    0, 0, sin(halfAngle), axis(3) * sin(halfAngle), axis(3) * sin(halfAngle)
];
initialRates = zeros(3, 5);
initialRates(:,5) = deg2rad([1; -0.5; 0.75]);

for k = 1:size(references,2)
    result = simulateAttitudeSlew(references(:,k), initialRates(:,k), 180);
    m = result.metrics;
    assert(m.finalPointingErrorDeg < 0.01);
    assert(~isnan(m.settlingTimeSec) && m.settlingTimeSec < 180);
    assert(isfinite(m.overshootDeg));
    assert(all(isfinite(m.controlEffortNms)));
    assert(all(isfinite(m.peakWheelMomentumNms)));
    assert(max(m.peakWheelSpeedRadPerSec - rw.maxSpeed') < 1e-6);
    peakAppliedTorque = max(max(abs(result.appliedMotorTorque)));
    assert(peakAppliedTorque <= max(rw.maxTorque) + 1e-15);
end

stationary = simulateAttitudeSlew([1; 0; 0; 0], zeros(3,1), 1);
assert(stationary.metrics.finalPointingErrorDeg == 0);
assert(stationary.metrics.settlingTimeSec == 0);
assert(stationary.metrics.overshootDeg == 0);

%% Representative solver convergence

loose = simulateAttitudeSlew( ...
    references(:,4), zeros(3,1), 180, ...
    odeset("RelTol", 1e-7, "AbsTol", 1e-9));
tight = simulateAttitudeSlew( ...
    references(:,4), zeros(3,1), 180, ...
    odeset("RelTol", 1e-9, "AbsTol", 1e-11));
qDifference = quaternionAttitudeError( ...
    tight.x(end,1:4)', loose.x(end,1:4)');
finalDifferenceDeg = rad2deg(2 * acos(max(-1, min(1, qDifference(1)))));
assert(finalDifferenceDeg < 1e-4);

disp("ALL ATTITUDE SLEW TESTS PASSED");
