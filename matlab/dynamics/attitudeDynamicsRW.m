function xdot = attitudeDynamicsRW(~, x, sc, rw, motorTorque)
% ATTITUDEDYNAMICSRW Rigid-body dynamics with one X-axis reaction wheel.

% State:
% x(1:4) = q_IB
% x(5:7) = omega_BI^B
% x(8)   = reaction wheel angular velocity

% motorTorque:
% torque applied to the reaction wheel motor [N*m]

    %% Extract state
    
    q = x(1:4);
    omega = x(5:7);
    wheelSpeed = x(8);
    
    %% Spacecraft inertia
    
    J = sc.J;
    
    %% Saturate motor torque

    wheelInertia = rw.J(1);
    maxTorque = rw.maxTorque(1);
    
    motorTorque = max( ...
        min(motorTorque, maxTorque), ...
        -maxTorque);
    
    %% Reaction wheel acceleration
    
    wheelAccel = motorTorque / wheelInertia;
    
    %% Torque applied to spacecraft
    
    bodyTorque = [
        -motorTorque
        0
        0
    ];
    
    %% Quaternion kinematics
    
    omegaQuat = [
        0
        omega
    ];
    
    qdot = 0.5 * quatMultiply(q, omegaQuat);
    
    %% Spacecraft rotational dynamics
    
    Hbody = J * omega;
    
    gyroscopicTerm = cross(omega, Hbody);
    
    omegaDot = ...
        J \ (bodyTorque - gyroscopicTerm);
    
    %% State derivative
    
    xdot = [
        qdot
        omegaDot
        wheelAccel
    ];

end
