function xdot = integratedSpacecraftDynamics(time,x,sc,rw,mtq,earth,model,motorCommand,dipoleCommand)
% INTEGRATEDSPACECRAFTDYNAMICS 6-DOF truth plant with wheels and rods.
% State is [r_I (m); v_I (m/s); q_IB; omega_BI_B (rad/s); wheelSpeed (rad/s)].
% Commands are wheel motor torque [N m] and body-frame rod dipole [A m^2].
    position=x(1:3); velocity=x(4:6); q=x(7:10);
    omega=x(11:13); wheelSpeed=x(14:16);
    acceleration=twoBodyAcceleration(position,earth); disturbance=zeros(3,1);

    if model.includeJ2, acceleration=acceleration+j2Acceleration(position,earth); end
    if model.includeDrag || model.includeAerodynamicTorque
        [force,torque]=aerodynamicDisturbance(position,velocity,q,sc,earth);
        if model.includeDrag, acceleration=acceleration+force/sc.mass; end
        if model.includeAerodynamicTorque, disturbance=disturbance+torque; end
    end
    if model.includeSolarPressure || model.includeSolarPressureTorque
        [force,torque]=solarRadiationDisturbance(position,q,time,sc,earth);
        if model.includeSolarPressure, acceleration=acceleration+force/sc.mass; end
        if model.includeSolarPressureTorque, disturbance=disturbance+torque; end
    end
    if model.includeGravityGradient
        disturbance=disturbance+gravityGradientTorque(position,q,sc.J,earth.mu);
    end
    if model.includeMagneticTorque
        disturbance=disturbance+magneticTorque(position,q,time,sc.residualDipoleB,earth);
    end

    fieldB=quatToDCM(q)'*earthMagneticField(position,time,earth);
    rodTorque=magnetorquerModel(dipoleCommand,fieldB,mtq);
    motorTorque=limitReactionWheelTorque(motorCommand,wheelSpeed,rw);
    wheelMomentum=rw.A*(rw.J.*wheelSpeed);
    totalMomentum=sc.J*omega+wheelMomentum;
    qdot=.5*quatMultiply(q,[0;omega]);
    omegaDot=sc.J\(disturbance+rodTorque-rw.A*motorTorque-cross(omega,totalMomentum));
    xdot=[velocity;acceleration;qdot;omegaDot;motorTorque./rw.J];
end
