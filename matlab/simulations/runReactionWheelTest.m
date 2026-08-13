clear;
clc;
close all;

%% Add project folders

addpath("../config");
addpath("../math");
addpath("../dynamics");

%% Load parameters

sc = spacecraftParams();
rw = reactionWheelParams();

%% Initial spacecraft attitude

q0 = [
    1
    0
    0
    0
];

%% Initial spacecraft angular velocity

omega0 = [
    0
    0
    0
];

%% Initial reaction wheel speed

wheelSpeed0 = 0;

%% Initial state

x0 = [
    q0
    omega0
    wheelSpeed0
];

%% Motor torque applied to wheel

motorTorque = 1e-4;

%% Simulation time

tspan = [0 10];

%% Dynamics function

dynamicsFcn = @(t,x) attitudeDynamicsRW( ...
    t, ...
    x, ...
    sc, ...
    rw, ...
    motorTorque);

%% Run simulation

[t, x] = ode45(dynamicsFcn, tspan, x0);

%% Extract states

q = x(:,1:4);
omega = x(:,5:7);
wheelSpeed = x(:,8);

%% Normalize quaternions

for k = 1:size(q,1)
    q(k,:) = quatNormalize(q(k,:)')';
end

%% Plot spacecraft angular velocity

figure;

plot(t, rad2deg(omega), "LineWidth", 1.2);

xlabel("Time [s]");
ylabel("Spacecraft Angular Velocity [deg/s]");

legend("\omega_x", "\omega_y", "\omega_z");

title("Spacecraft Angular Velocity");

grid on;


%% Plot reaction wheel speed

figure;

plot(t, wheelSpeed, "LineWidth", 1.2);

xlabel("Time [s]");
ylabel("Reaction Wheel Speed [rad/s]");

title("Reaction Wheel Speed");

grid on;

%% Verify total angular momentum conservation

spacecraftMomentum = ...
    sc.J(1,1) .* omega(:,1);

wheelMomentum = ...
    rw.J .* wheelSpeed;

totalMomentum = ...
    spacecraftMomentum + wheelMomentum;

figure;

plot(t, totalMomentum, "LineWidth", 1.2);

xlabel("Time [s]");
ylabel("Total X Angular Momentum [N*m*s]");

title("Spacecraft + Reaction Wheel Momentum");

grid on;

fprintf( ...
    "Maximum total momentum error: %.3e N*m*s\n", ...
    max(abs(totalMomentum)));