function acceleration = j2Acceleration(positionECI, earth)
% J2ACCELERATION First-order Earth oblateness perturbation in ECI [m/s^2].

    x = positionECI(1);
    y = positionECI(2);
    z = positionECI(3);
    radius = norm(positionECI);
    zRatioSquared = (z / radius)^2;
    factor = 1.5 * earth.J2 * earth.mu * earth.radius^2 / radius^5;

    acceleration = factor * [ ...
        x * (5 * zRatioSquared - 1)
        y * (5 * zRatioSquared - 1)
        z * (5 * zRatioSquared - 3)
    ];

end
