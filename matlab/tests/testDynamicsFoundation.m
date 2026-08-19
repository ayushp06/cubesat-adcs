clear;
clc;

testDir = fileparts(mfilename("fullpath"));
addpath(fullfile(testDir, "..", "config"));
addpath(fullfile(testDir, "..", "math"));
addpath(fullfile(testDir, "..", "dynamics"));

sc = spacecraftParams();
rw = reactionWheelParams();
options = odeset("RelTol", 1e-10, "AbsTol", 1e-12);

%% General torque-free motion conserves energy and inertial momentum

x0 = [1; 0; 0; 0; deg2rad([3; -2; 1])];
[~, x] = ode45( ...
    @(t, state) attitudeDynamics(t, state, sc, zeros(3,1)), ...
    [0, 50], x0, options);

energy = zeros(size(x,1), 1);
inertialMomentum = zeros(size(x,1), 3);

for k = 1:size(x,1)
    q = quatNormalize(x(k,1:4)');
    omega = x(k,5:7)';
    Hbody = sc.J * omega;
    energy(k) = 0.5 * omega' * Hbody;
    inertialMomentum(k,:) = (quatToDCM(q) * Hbody)';
end

assert(max(abs((energy - energy(1)) / energy(1))) < 1e-8);
assert(max(vecnorm(inertialMomentum - inertialMomentum(1,:), 2, 2)) < 1e-9);

%% Torque clipping and speed limits

command = [2; -2; 0.5] .* rw.maxTorque;
speed = [0; 0; 0];
assert(norm(limitReactionWheelTorque(command, speed, rw) - ...
    [rw.maxTorque(1); -rw.maxTorque(2); 0.5 * rw.maxTorque(3)]) < 1e-15);

speed = [rw.maxSpeed(1); -rw.maxSpeed(2); rw.maxSpeed(3)];
command = [rw.maxTorque(1); -rw.maxTorque(2); -rw.maxTorque(3)];
assert(norm(limitReactionWheelTorque(command, speed, rw) - ...
    [0; 0; -rw.maxTorque(3)]) < 1e-15);

%% Internal three-wheel torque conserves total angular momentum

x0 = [1; 0; 0; 0; zeros(6,1)];
command = [1e-4; -5e-5; 8e-5];
[~, x] = ode45( ...
    @(t, state) attitudeDynamics3RW(t, state, sc, rw, command), ...
    [0, 10], x0, options);

totalMomentum = x(:,5:7) * sc.J' + x(:,8:10) .* rw.J';
assert(max(vecnorm(totalMomentum, 2, 2)) < 1e-10);

disp("ALL DYNAMICS FOUNDATION TESTS PASSED");
