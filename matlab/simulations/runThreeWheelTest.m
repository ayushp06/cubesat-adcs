clear;
clc;
close all;

addpath("../config");
addpath("../math");
addpath("../dynamics");
addpath("../tests");

sc = spacecraftParams();
rw = reactionWheelParams();

%% Initial attitude

q0 = [
    1
    0
    0
    0
];

%% Initial spacecraft rate

omega0 = zeros(3,1);

%% Initial wheel speeds

wheelSpeed0 = zeros(3,1);

%% Initial state

x0 = [
    q0
    omega0
    wheelSpeed0
];

%% Wheel motor torques

motorTorque = [
    1e-4
    -5e-5
    8e-5
];

%% Simulation

tspan = [0 10];

dynamicsFcn = @(t,x) ...
    attitudeDynamics3RW( ...
        t, ...
        x, ...
        sc, ...
        rw, ...
        motorTorque);

[t,x] = ode45( ...
    dynamicsFcn, ...
    tspan, x0, ...
    odeset("RelTol", 1e-8, "AbsTol", 1e-10));

%% Extract

q = x(:,1:4);

omega = x(:,5:7);

wheelSpeed = x(:,8:10);

%% Plot spacecraft rates

figure;

plot( ...
    t, ...
    rad2deg(omega), ...
    "LineWidth", ...
    1.2);

xlabel("Time [s]");
ylabel("Body Rate [deg/s]");

legend( ...
    "\omega_x", ...
    "\omega_y", ...
    "\omega_z");

title("CubeSat Angular Velocity");

grid on;


%% Plot reaction-wheel speeds

figure;

plot( ...
    t, ...
    wheelSpeed, ...
    "LineWidth", ...
    1.2);

xlabel("Time [s]");
ylabel("Wheel Speed [rad/s]");

legend( ...
    "RW-X", ...
    "RW-Y", ...
    "RW-Z");

title("Reaction Wheel Speeds");

grid on;
