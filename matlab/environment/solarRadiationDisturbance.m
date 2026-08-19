function [forceECI, torqueB] = solarRadiationDisturbance(positionECI, qIB, time, sc, earth)
% SOLARRADIATIONDISTURBANCE Cannonball-pressure force with box projected area.
    [sunDirectionECI, distance] = sunVectorECI(time, earth);
    illumination = eclipseFactor(positionECI, sunDirectionECI, earth);
    dcmIB = quatToDCM(quatNormalize(qIB));
    area = projectedBoxArea(dcmIB' * sunDirectionECI, sc.dimensions);
    pressure = earth.solarPressure * (earth.astronomicalUnit / distance)^2;
    forceECI = -illumination * pressure * sc.reflectivityCoefficient * ...
        area * sunDirectionECI;
    torqueB = cross(sc.centerOfPressureB, dcmIB' * forceECI);
end
