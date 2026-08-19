function acceleration = twoBodyAcceleration(positionECI, earth)
% TWOBODYACCELERATION Point-mass Earth gravity in ECI [m/s^2].

    radius = norm(positionECI);
    acceleration = -earth.mu * positionECI / radius^3;

end
