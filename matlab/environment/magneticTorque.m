function torqueB = magneticTorque(positionECI, qIB, time, dipoleB, earth)
% MAGNETICTORQUE Residual spacecraft dipole crossed with geomagnetic field.
    fieldB = quatToDCM(quatNormalize(qIB))' * ...
        earthMagneticField(positionECI, time, earth);
    torqueB = cross(dipoleB, fieldB);
end
