clear;
clc;
close all;

simulationDir = fileparts(mfilename("fullpath"));
addpath(fullfile(simulationDir, "..", "config"));
addpath(fullfile(simulationDir, "..", "control"));
addpath(fullfile(simulationDir, "..", "dynamics"));
addpath(fullfile(simulationDir, "..", "math"));

halfAngle = pi / 4;
axis = [1; 2; 3] / norm([1; 2; 3]);
caseNames = {"X90", "Y90", "Z90", "Arbitrary", "ArbitraryWithRate"};
references = [ ...
    cos(halfAngle), cos(halfAngle), cos(halfAngle), cos(halfAngle), cos(halfAngle)
    sin(halfAngle), 0, 0, axis(1) * sin(halfAngle), axis(1) * sin(halfAngle)
    0, sin(halfAngle), 0, axis(2) * sin(halfAngle), axis(2) * sin(halfAngle)
    0, 0, sin(halfAngle), axis(3) * sin(halfAngle), axis(3) * sin(halfAngle)
];
initialRates = zeros(3, 5);
initialRates(:,5) = deg2rad([1; -0.5; 0.75]);
results = cell(1, numel(caseNames));

fprintf("Case                 Error(deg)  Settle(s)  Overshoot(deg)  PeakRate(deg/s)\n");

for k = 1:numel(caseNames)
    results{k} = simulateAttitudeSlew( ...
        references(:,k), initialRates(:,k), 180);
    m = results{k}.metrics;
    fprintf("%-20s %10.4f %10.1f %15.4f %16.4f\n", ...
        caseNames{k}, m.finalPointingErrorDeg, m.settlingTimeSec, ...
        m.overshootDeg, m.peakBodyRateDegPerSec);
    fprintf("  effort [N m s]: [%g %g %g], peak wheel speed [rad/s]: [%g %g %g]\n", ...
        m.controlEffortNms, m.peakWheelSpeedRadPerSec);
    fprintf("  peak wheel momentum [N m s]: [%g %g %g], total peak: %g\n", ...
        m.peakWheelMomentumNms, m.peakTotalWheelMomentumNms);
end

figure;
hold on;
for k = 1:numel(caseNames)
    errorDeg = zeros(size(results{k}.t));
    for j = 1:numel(errorDeg)
        qError = quaternionAttitudeError( ...
            results{k}.qReference, results{k}.x(j,1:4)');
        errorDeg(j) = rad2deg(2 * acos(max(-1, min(1, qError(1)))));
    end
    plot(results{k}.t, errorDeg, "LineWidth", 1.2);
end
xlabel("Time [s]");
ylabel("Pointing Error [deg]");
title("Closed-Loop Attitude Slews");
legend(caseNames);
grid on;
