clear;
clc;

testDir = fileparts(mfilename("fullpath"));
addpath(fullfile(testDir, "..", "config"));
addpath(fullfile(testDir, "..", "math"));
addpath(fullfile(testDir, "..", "dynamics"));

sc = spacecraftParams();
rw = reactionWheelParams();
motorTorque = 1e-4;
x0 = [1; 0; 0; 0; 0; 0; 0; 0];

xdot = attitudeDynamicsRW(0, x0, sc, rw, motorTorque);

assert(isequal(size(xdot), [8, 1]));
assert(abs(xdot(5) + motorTorque / sc.J(1,1)) < 1e-12);
assert(abs(xdot(8) - motorTorque / rw.J(1)) < 1e-12);

[~, x] = ode45( ...
    @(t, state) attitudeDynamicsRW(t, state, sc, rw, motorTorque), ...
    [0, 10], ...
    x0);

totalMomentum = sc.J(1,1) .* x(:,5) + rw.J(1) .* x(:,8);

assert(max(abs(totalMomentum - totalMomentum(1))) < 1e-12);
assert(abs(x(end,8) - 20) < 1e-10);

disp("ALL ONE-WHEEL DYNAMICS TESTS PASSED");
