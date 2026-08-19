clear;
clc;

testDir = fileparts(mfilename("fullpath"));
addpath(fullfile(testDir, "..", "config"));
addpath(fullfile(testDir, "..", "dynamics"));
addpath(fullfile(testDir, "..", "environment"));

earth = earthEnvironmentParams();
radius = earth.radius + 400e3;
speed = sqrt(earth.mu / radius);
period = 2 * pi * sqrt(radius^3 / earth.mu);
x0 = [radius; 0; 0; 0; speed; 0];
options = odeset("RelTol", 1e-10, "AbsTol", 1e-6);

[~, x] = ode45( ...
    @(t, state) orbitalDynamics(t, state, earth, false), ...
    linspace(0, period, 1001), x0, options);

specificEnergy = 0.5 * sum(x(:,4:6).^2, 2) - ...
    earth.mu ./ vecnorm(x(:,1:3), 2, 2);
angularMomentum = cross(x(:,1:3), x(:,4:6), 2);

assert(norm(x(end,1:3)' - x0(1:3)) < 1);
assert(norm(x(end,4:6)' - x0(4:6)) < 1e-3);
assert(max(abs((specificEnergy - specificEnergy(1)) / specificEnergy(1))) < 1e-9);
assert(max(vecnorm(angularMomentum - angularMomentum(1,:), 2, 2)) / ...
    norm(angularMomentum(1,:)) < 1e-9);

equator = [radius; 0; 0];
expectedEquator = -1.5 * earth.J2 * earth.mu * earth.radius^2 / radius^4;
equatorAcceleration = j2Acceleration(equator, earth);
assert(abs(equatorAcceleration(1) - expectedEquator) < 1e-15);

pole = [0; 0; radius];
expectedPole = 3 * earth.J2 * earth.mu * earth.radius^2 / radius^4;
poleAcceleration = j2Acceleration(pole, earth);
assert(abs(poleAcceleration(3) - expectedPole) < 1e-15);

disp("ALL ORBIT DYNAMICS TESTS PASSED");
