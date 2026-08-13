function xdot = attitudeDynamics(~, x, sc, tau)
% ATTITUDEDYNAMICS Nonlinear rigid-body rotational dynamics 

% State:
% x(1:4) = q_IB
% x(5:7) = omega_BI^B [rad/s]

% Inputs:
% sc.J = spacecraft inertia tensor [kg*m^2]
% tau  = applied body torque [N*m]

% Output:
% xdot = [qdot; omegaDot]

    %% Extract state

    q = x(1:4);
    omega = x(5:7);
    
    %% Spacecraft inertia
    
    J = sc.J;
    
    %% Quaternion kinematics
    
    omegaQuat = [
        0
        omega
    ];
    
    qdot = 0.5 * quatMultiply(q, omegaQuat);
    
    %% Euler rigid-body rotational dynamics
    
    angularMomentum = J * omega;
    
    gyroscopicTerm = cross(omega, angularMomentum);
    
    omegaDot = J \ (tau - gyroscopicTerm);
    
    %% State derivative
    
    xdot = [
        qdot
        omegaDot
    ];

end