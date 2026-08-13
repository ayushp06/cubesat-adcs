clear;
clc;
close all;

%% Add project folders

addpath("../config");
addpath("../math");
addpath("../dynamics");
addpath("../tests");

%% Load spacecraft

sc = spacecraftParams();

%% Initial attitude

q0 = [
    1
    0
    0
    0
];

%% Initial angular velocity

omega0_deg = [
    5
    0
    0
];

omega0 = deg2rad(omega0_deg);

%% Initial state

x0 = [
    q0
    omega0
];

%% Applied torque

tau = zeros(3,1);

%% Simulation time

tspan = [0 100];

%% Dynamics function

dynamicsFcn = @(t,x) attitudeDynamics( ...
    t, ...
    x, ...
    sc, ...
    tau);

%% Integrate

[t, x] = ode45(dynamicsFcn, tspan, x0);

%% Extract states

q = x(:,1:4);
omega = x(:,5:7);

%% Normalize quaternion output

for k = 1:size(q,1)
    q(k,:) = quatNormalize(q(k,:)')';
end


%% Plot angular velocity

figure;

plot(t, rad2deg(omega), "LineWidth", 1.2);

xlabel("Time [s]");
ylabel("Angular Velocity [deg/s]");

legend("\omega_x", "\omega_y", "\omega_z");

title("Torque-Free CubeSat Angular Velocity");

grid on;


%% Plot quaternion

figure;

plot(t, q, "LineWidth", 1.2);

xlabel("Time [s]");
ylabel("Quaternion Component");

legend("q_w", "q_x", "q_y", "q_z");

title("CubeSat Attitude Quaternion");

grid on;


%% STEP 5: Verify quaternion norm

quatNorm = vecnorm(q, 2, 2);

figure;

plot(t, quatNorm, "LineWidth", 1.2);

xlabel("Time [s]");
ylabel("||q||");

title("Quaternion Norm");

grid on;

maxQuatError = max(abs(quatNorm - 1));

fprintf( ...
    "Maximum quaternion norm error: %.3e\n", ...
    maxQuatError);


%% STEP 6: Verify rotational kinetic energy

energy = zeros(length(t),1);

for k = 1:length(t)

    omega_k = omega(k,:)';

    energy(k) = ...
        0.5 * omega_k' * sc.J * omega_k;

end

figure;

plot(t, energy, "LineWidth", 1.2);

xlabel("Time [s]");
ylabel("Rotational Kinetic Energy [J]");

title("Energy Conservation");

grid on;


%% Relative energy error

energyError = ...
    (energy - energy(1)) / energy(1);

figure;

plot(t, energyError, "LineWidth", 1.2);

xlabel("Time [s]");
ylabel("Relative Energy Error");

title("Numerical Energy Error");

grid on;


%% STEP 7: Verify angular momentum magnitude

Hmag = zeros(length(t),1);

for k = 1:length(t)

    omega_k = omega(k,:)';

    H = sc.J * omega_k;

    Hmag(k) = norm(H);

end

figure;

plot(t, Hmag, "LineWidth", 1.2);

xlabel("Time [s]");
ylabel("|H| [N*m*s]");

title("Angular Momentum Magnitude");

grid on;


%% Relative angular momentum error

momentumError = ...
    (Hmag - Hmag(1)) / Hmag(1);

figure;

plot(t, momentumError, "LineWidth", 1.2);

xlabel("Time [s]");
ylabel("Relative Momentum Error");

title("Angular Momentum Conservation Error");

grid on;