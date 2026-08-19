function earth = earthEnvironmentParams()
% EARTHENVIRONMENTPARAMS Earth and space-environment constants in SI units.

    earth.mu = 3.986004418e14;          % [m^3/s^2]
    earth.radius = 6378137;             % WGS-84 equatorial radius [m]
    earth.J2 = 1.08262668e-3;           % [-]
    earth.rotationRate = 7.2921150e-5;  % [rad/s]
    earth.atmosphere.referenceAltitude = 400e3; % [m]
    earth.atmosphere.referenceDensity = 3.725e-12; % [kg/m^3]
    earth.atmosphere.scaleHeight = 58.515e3; % [m]
    earth.solarPressure = 4.56e-6;      % [N/m^2] at 1 AU
    earth.astronomicalUnit = 149597870700; % [m]
    earth.year = 365.25 * 86400;        % [s]
    earth.obliquity = deg2rad(23.43928); % [rad]
    earth.magneticDipoleECEF = 7.94e22 * ...
        [sin(deg2rad(9.3)); 0; cos(deg2rad(9.3))]; % [A m^2]

end
