function torqueB = gravityGradientTorque(positionECI, qIB, inertia, mu)
% GRAVITYGRADIENTTORQUE Point-mass gravity-gradient torque [N m].
    radius = norm(positionECI);
    radialB = quatToDCM(quatNormalize(qIB))' * positionECI / radius;
    torqueB = 3 * mu / radius^3 * cross(radialB, inertia * radialB);
end
