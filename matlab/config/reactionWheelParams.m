function rw = reactionWheelParams()
% REACTIONWHEELPARAMS Three-axis reaction-wheel assembly.

    %% Wheel inertia [kg*m^2]

    rw.J = [
        5e-5
        5e-5
        5e-5
    ];

    %% Maximum motor torque [N*m]

    rw.maxTorque = [
        2e-4
        2e-4
        2e-4
    ];

    %% Maximum wheel speed [rad/s]

    maxRPM = 6000;

    rw.maxSpeed = ...
        maxRPM * 2*pi/60 * ones(3,1);

    %% Wheel-axis matrix

    % Columns represent wheel spin axes
    % expressed in spacecraft body coordinates.

    rw.A = eye(3);

end