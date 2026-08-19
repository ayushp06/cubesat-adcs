clear; clc;
testDir = fileparts(mfilename("fullpath"));
addpath(fullfile(testDir, "..", "config"));
addpath(fullfile(testDir, "..", "environment"));
addpath(fullfile(testDir, "..", "math"));

earth = earthEnvironmentParams(); sc = spacecraftParams();
r = [earth.radius + 400e3; 0; 0]; q = [1; 0; 0; 0];

assert(norm(gravityGradientTorque(r, q, diag([1, 2, 3]), earth.mu)) < 1e-20);
qz45 = [cos(pi/8); 0; 0; sin(pi/8)];
tauGG = gravityGradientTorque(r, qz45, diag([1, 2, 3]), earth.mu);
assert(abs(tauGG(3) + 1.5*earth.mu/norm(r)^3) < 1e-18);
assert(abs(atmosphericDensity(r, earth) - earth.atmosphere.referenceDensity) < 1e-25);

v = [0; sqrt(earth.mu/norm(r)); 0];
[drag, dragTorque] = aerodynamicDisturbance(r, v, q, sc, earth);
assert(dot(drag, v - cross([0;0;earth.rotationRate], r)) < 0);
assert(norm(dragTorque - cross(sc.centerOfPressureB, drag)) < 1e-20);

[sun, distance] = sunVectorECI(0, earth);
assert(norm(sun - [1;0;0]) < 1e-15 && distance == earth.astronomicalUnit);
assert(eclipseFactor(r, sun, earth) == 1);
assert(eclipseFactor(-r, sun, earth) == 0);
[srpLit, ~] = solarRadiationDisturbance(r, q, 0, sc, earth);
[srpDark, ~] = solarRadiationDisturbance(-r, q, 0, sc, earth);
assert(norm(srpLit) > 0 && norm(srpDark) == 0);

fieldEquator = earthMagneticField(r, 0, earth);
fieldPole = earthMagneticField([0;0;norm(r)], 0, earth);
assert(norm(fieldPole) > norm(fieldEquator));
assert(norm(magneticTorque(r, q, 0, sc.residualDipoleB, earth) - ...
    cross(sc.residualDipoleB, fieldEquator)) < 1e-20);
disp("ALL SPACE ENVIRONMENT TESTS PASSED");
