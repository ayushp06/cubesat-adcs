clear;
clc;

testDir = fileparts(mfilename("fullpath"));
addpath(fullfile(testDir, "..", "config"));
addpath(fullfile(testDir, "..", "control"));
addpath(fullfile(testDir, "..", "dynamics"));
addpath(fullfile(testDir, "..", "math"));

sc = spacecraftParams();
rw = reactionWheelParams();
controller = controllerParams(sc);
qIdentity = [1; 0; 0; 0];
qX90 = [cos(pi/4); sin(pi/4); 0; 0];

%% Quaternion error follows the documented convention and shortest path

assert(norm(quaternionAttitudeError(qIdentity, qIdentity) - qIdentity) < 1e-12);
expectedError = [cos(pi/4); -sin(pi/4); 0; 0];
assert(norm(quaternionAttitudeError(qX90, qIdentity) - expectedError) < 1e-12);
assert(norm(quaternionAttitudeError(-qX90, qIdentity) - expectedError) < 1e-12);

%% Proportional torque moves identity attitude toward +90 degrees about X

bodyTorque = quaternionPDController( ...
    qX90, qIdentity, zeros(3,1), controller);
assert(bodyTorque(1) > 0);
assert(norm(bodyTorque(2:3)) < 1e-15);

%% Derivative torque opposes body rate

bodyTorque = quaternionPDController( ...
    qIdentity, qIdentity, [0.01; -0.02; 0.03], controller);
assert(all(bodyTorque .* [0.01; -0.02; 0.03] < 0));

%% Wheel allocation produces equal-and-opposite body torque

requestedTorque = [1e-4; -5e-5; 8e-5];
motorTorque = allocateReactionWheelTorque(requestedTorque, rw);
assert(norm(-rw.A * motorTorque - requestedTorque) < 1e-15);

disp("ALL ATTITUDE CONTROL TESTS PASSED");
