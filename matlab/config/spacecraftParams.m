function sc = spacecraftParams()
% CubeSat Parameters for ADCS digital twin

%% Units:
% SI units throughout the project:
% length       -> meters
% mass         -> kilograms
% time         -> seconds
% angle        -> radians
% angular rate -> radians/second
% torque       -> N*m

%% Spacecraft configuration

    sc.name = "3U CubeSat";

    % Mass [kg]
    sc.mass = 4.0;

    % Approximate dimensions [m]
    sc.dimensions = [
        0.10
        0.10
        0.30
    ];

    %% Inertia tensor

    % Initial nominal inertia tensor [kg*m^2]
    %
    % Body frame:
    % +X = spacecraft right
    % +Y = spacecraft forward
    % +Z = spacecraft up

    sc.J = [
        0.034  0      0
        0      0.030  0
        0      0      0.007
    ];

    sc.dragCoefficient = 2.2;
    sc.reflectivityCoefficient = 1.3;
    sc.centerOfPressureB = [0.01; 0; 0.02]; % relative to COM [m]
    sc.residualDipoleB = [0.01; -0.005; 0.002]; % [A m^2]


end
