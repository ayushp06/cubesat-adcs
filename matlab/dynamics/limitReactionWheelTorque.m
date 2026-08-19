function motorTorque = limitReactionWheelTorque(command, wheelSpeed, rw)
% LIMITREACTIONWHEELTORQUE Apply motor-torque and directional speed limits.

    motorTorque = max(min(command, rw.maxTorque), -rw.maxTorque);

    outward = ...
        (wheelSpeed >= rw.maxSpeed & motorTorque > 0) | ...
        (wheelSpeed <= -rw.maxSpeed & motorTorque < 0);

    motorTorque(outward) = 0;

end
