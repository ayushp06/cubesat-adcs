function xdot = attitudeDynamics3RW( ...
    ~, x, sc, rw, motorTorque)
% ATTITUDEDYNAMICS3RW
% Nonlinear CubeSat rotational dynamics with 3 reaction wheels.

% State:
% x(1:4)  = q_IB
% x(5:7)  = spacecraft body rate [rad/s]
% x(8:10) = reaction wheel speeds [rad/s]

% motorTorque:
% commanded torque applied to the wheels [N*m]

    %% Extract states
    
    q = x(1:4);
    
    omega = x(5:7);
    
    wheelSpeed = x(8:10);
    
    %% Saturate motor torque
    
    motorTorque = max( ...
        min(motorTorque, rw.maxTorque), ...
        -rw.maxTorque);
    
    %% Reaction-wheel acceleration
    
    wheelAccel = motorTorque ./ rw.J;
    
    %% Wheel angular momentum
    
    wheelMomentum = ...
        rw.J .* wheelSpeed;
    
    %% Express wheel momentum in body coordinates
    
    HwheelBody = ...
        rw.A * wheelMomentum;
    
    %% Spacecraft rigid-body angular momentum
    
    Hbody = ...
        sc.J * omega;
    
    %% Total internal angular momentum
    
    HtotalBody = ...
        Hbody + HwheelBody;
    
    %% Reaction torque on spacecraft
    
    bodyTorque = ...
        -rw.A * motorTorque;
    
    %% Quaternion kinematics
    
    omegaQuat = [
        0
        omega
    ];
    
    qdot = ...
        0.5 * quatMultiply(q, omegaQuat);
    
    %% Rotational dynamics
    
    gyroscopicTerm = ...
        cross(omega, HtotalBody);
    
    omegaDot = ...
        sc.J \ (bodyTorque - gyroscopicTerm);
    
    %% State derivative
    
    xdot = [
        qdot
        omegaDot
        wheelAccel
    ];

end