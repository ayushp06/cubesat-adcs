clear; clc;
testDir = fileparts(mfilename("fullpath"));
addpath(fullfile(testDir, "..", "config"));
addpath(fullfile(testDir, "..", "dynamics"));
addpath(fullfile(testDir, "..", "environment"));
addpath(fullfile(testDir, "..", "math"));

earth = earthEnvironmentParams(); sc = spacecraftParams();
model = truthModelParams();
fields = fieldnames(model);
for index = 1:numel(fields), model.(fields{index}) = false; end
radius = earth.radius + 400e3; speed = sqrt(earth.mu/radius);
x0 = [radius;0;0; 0;speed;0; 1;0;0;0; 0;0;0.01];
derivative = fullSpacecraftDynamics(0, x0, sc, earth, model);
assert(norm(derivative(1:6) - [x0(4:6); twoBodyAcceleration(x0(1:3), earth)]) < 1e-12);
assert(norm(derivative(7:13) - attitudeDynamics(0, x0(7:13), sc, zeros(3,1))) < 1e-12);

period = 2*pi*sqrt(radius^3/earth.mu);
options = odeset("RelTol",1e-10,"AbsTol",1e-8);
[~, state] = ode45(@(t,x) fullSpacecraftDynamics(t,x,sc,earth,model), ...
    linspace(0,period,501), x0, options);
energy = 0.5*sum(state(:,4:6).^2,2) - earth.mu./vecnorm(state(:,1:3),2,2);
momentum = cross(state(:,1:3),state(:,4:6),2);
assert(max(abs((energy-energy(1))/energy(1))) < 1e-9);
assert(max(vecnorm(momentum-momentum(1,:),2,2))/norm(momentum(1,:)) < 1e-9);
assert(max(abs(vecnorm(state(:,7:10),2,2)-1)) < 1e-7);

model = truthModelParams();
assert(all(isfinite(fullSpacecraftDynamics(123, x0, sc, earth, model))));
disp("ALL FULL SPACECRAFT DYNAMICS TESTS PASSED");
