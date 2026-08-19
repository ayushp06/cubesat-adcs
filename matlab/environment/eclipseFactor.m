function illumination = eclipseFactor(positionECI, sunDirectionECI, earth)
% ECLIPSEFACTOR Binary cylindrical Earth-shadow model (0 shadow, 1 Sun).
    behindEarth = dot(positionECI, sunDirectionECI) < 0;
    axisDistance = norm(positionECI - dot(positionECI, sunDirectionECI) * sunDirectionECI);
    illumination = double(~(behindEarth && axisDistance < earth.radius));
end
