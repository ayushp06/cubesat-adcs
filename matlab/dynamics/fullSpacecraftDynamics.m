function xdot = fullSpacecraftDynamics(time, x, sc, earth, model)
% FULLSPACECRAFTDYNAMICS Coupled 13-state ECI translation and body rotation.
% x = [r_I (m); v_I (m/s); q_IB; omega_BI_B (rad/s)].
    position = x(1:3); velocity = x(4:6); q = x(7:10); omega = x(11:13);
    acceleration = twoBodyAcceleration(position, earth);
    torqueB = zeros(3, 1);

    if model.includeJ2
        acceleration = acceleration + j2Acceleration(position, earth);
    end
    if model.includeDrag || model.includeAerodynamicTorque
        [dragForce, dragTorque] = aerodynamicDisturbance(position, velocity, q, sc, earth);
        if model.includeDrag, acceleration = acceleration + dragForce / sc.mass; end
        if model.includeAerodynamicTorque, torqueB = torqueB + dragTorque; end
    end
    if model.includeSolarPressure || model.includeSolarPressureTorque
        [solarForce, solarTorque] = solarRadiationDisturbance(position, q, time, sc, earth);
        if model.includeSolarPressure, acceleration = acceleration + solarForce / sc.mass; end
        if model.includeSolarPressureTorque, torqueB = torqueB + solarTorque; end
    end
    if model.includeGravityGradient
        torqueB = torqueB + gravityGradientTorque(position, q, sc.J, earth.mu);
    end
    if model.includeMagneticTorque
        torqueB = torqueB + magneticTorque(position, q, time, sc.residualDipoleB, earth);
    end

    attitudeDerivative = attitudeDynamics(time, [q; omega], sc, torqueB);
    xdot = [velocity; acceleration; attitudeDerivative];
end
