function [sunDirectionECI, earthSunDistance] = sunVectorECI(time, earth)
% SUNVECTORECI Circular-Earth-orbit analytic Sun direction and distance.
    longitude = 2 * pi * time / earth.year;
    sunDirectionECI = [cos(longitude); ...
        cos(earth.obliquity)*sin(longitude); ...
        sin(earth.obliquity)*sin(longitude)];
    earthSunDistance = earth.astronomicalUnit;
end
