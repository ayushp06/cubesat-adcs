clear;
clc;
close all;

simulationDir = fileparts(mfilename("fullpath"));
addpath(fullfile(simulationDir, "..", "config"));
addpath(fullfile(simulationDir, "..", "dynamics"));
addpath(fullfile(simulationDir, "..", "environment"));

earth = earthEnvironmentParams();
radius = earth.radius + 400e3;
speed = sqrt(earth.mu / radius);
period = 2 * pi * sqrt(radius^3 / earth.mu);
inclination = deg2rad(51.6);
x0 = [radius; 0; 0; 0; speed * cos(inclination); speed * sin(inclination)];
times = linspace(0, period, 1001);
options = odeset("RelTol", 1e-10, "AbsTol", 1e-6);

[t, twoBody] = ode45( ...
    @(time, state) orbitalDynamics(time, state, earth, false), ...
    times, x0, options);
[~, withJ2] = ode45( ...
    @(time, state) orbitalDynamics(time, state, earth, true), ...
    times, x0, options);

energy = 0.5 * sum(twoBody(:,4:6).^2, 2) - ...
    earth.mu ./ vecnorm(twoBody(:,1:3), 2, 2);
momentum = cross(twoBody(:,1:3), twoBody(:,4:6), 2);
energyError = max(abs((energy - energy(1)) / energy(1)));
momentumError = max(vecnorm(momentum - momentum(1,:), 2, 2)) / ...
    norm(momentum(1,:));

fprintf("Circular-orbit period: %.3f s\n", period);
fprintf("Maximum relative specific-energy error: %.3e\n", energyError);
fprintf("Maximum relative angular-momentum error: %.3e\n", momentumError);
fprintf("J2 endpoint displacement after one orbit: %.3f m\n", ...
    norm(withJ2(end,1:3) - twoBody(end,1:3)));

figure;
plot3(twoBody(:,1), twoBody(:,2), twoBody(:,3), "LineWidth", 1.2);
hold on;
plot3(withJ2(:,1), withJ2(:,2), withJ2(:,3), "LineWidth", 1.2);
axis equal;
xlabel("ECI X [m]");
ylabel("ECI Y [m]");
zlabel("ECI Z [m]");
legend("Two body", "Two body + J2");
title("400 km, 51.6 deg Orbit");
grid on;
