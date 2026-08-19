function xdot = orbitalDynamics(~, x, earth, includeJ2)
% ORBITALDYNAMICS Propagate ECI position and velocity in SI units.
% State: x = [r_ECI (m); v_ECI (m/s)].

    positionECI = x(1:3);
    velocityECI = x(4:6);
    accelerationECI = twoBodyAcceleration(positionECI, earth);

    if includeJ2
        accelerationECI = accelerationECI + ...
            j2Acceleration(positionECI, earth);
    end

    xdot = [velocityECI; accelerationECI];

end
