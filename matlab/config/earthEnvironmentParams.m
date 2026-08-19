function earth = earthEnvironmentParams()
% EARTHENVIRONMENTPARAMS Earth and space-environment constants in SI units.

    earth.mu = 3.986004418e14;          % [m^3/s^2]
    earth.radius = 6378137;             % WGS-84 equatorial radius [m]
    earth.J2 = 1.08262668e-3;           % [-]
    earth.rotationRate = 7.2921150e-5;  % [rad/s]

end
