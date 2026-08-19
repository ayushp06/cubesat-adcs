function [forceECI, torqueB] = aerodynamicDisturbance(positionECI, velocityECI, qIB, sc, earth)
% AERODYNAMICDISTURBANCE Co-rotating exponential-atmosphere drag and torque.
    atmosphereVelocity = cross([0; 0; earth.rotationRate], positionECI);
    relativeVelocityECI = velocityECI - atmosphereVelocity;
    speed = norm(relativeVelocityECI);
    dcmIB = quatToDCM(quatNormalize(qIB));
    area = projectedBoxArea(dcmIB' * relativeVelocityECI, sc.dimensions);
    forceECI = -0.5 * atmosphericDensity(positionECI, earth) * ...
        sc.dragCoefficient * area * speed * relativeVelocityECI;
    torqueB = cross(sc.centerOfPressureB, dcmIB' * forceECI);
end
