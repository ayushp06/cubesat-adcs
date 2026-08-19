function density = atmosphericDensity(positionECI, earth)
% ATMOSPHERICDENSITY Single-scale-height LEO atmosphere [kg/m^3].
    altitude = norm(positionECI) - earth.radius;
    atmosphere = earth.atmosphere;
    density = atmosphere.referenceDensity * exp( ...
        -(altitude - atmosphere.referenceAltitude) / atmosphere.scaleHeight);
end
