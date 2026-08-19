function fieldECI = earthMagneticField(positionECI, time, earth)
% EARTHMAGNETICFIELD Rotating centered tilted-dipole field [T].
    angle = earth.rotationRate * time;
    rotation = [cos(angle), -sin(angle), 0; ...
                sin(angle),  cos(angle), 0; 0, 0, 1];
    dipoleECI = rotation * earth.magneticDipoleECEF;
    radial = positionECI / norm(positionECI);
    fieldECI = 1e-7 / norm(positionECI)^3 * ...
        (3 * radial * dot(dipoleECI, radial) - dipoleECI);
end
