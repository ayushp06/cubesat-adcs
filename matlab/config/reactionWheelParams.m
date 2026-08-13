function rw = reactionWheelParams()
% REACTIONWHEELPARAMS Nominal single reaction wheel parameters.

    % Wheel inertia [kg*m^2]
    rw.J = 5e-5;
    
    % Maximum wheel torque [N*m]
    rw.maxTorque = 2e-4;
    
    % Maximum wheel speed [rad/s]
    rw.maxSpeed = 6000 * 2*pi/60;

end