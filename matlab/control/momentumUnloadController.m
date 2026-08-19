function [motorTorque,dipoleCommand,magneticTorque] = momentumUnloadController(wheelSpeed,fieldB,rw,mtq)
% MOMENTUMUNLOADCONTROLLER Magnetically balanced reaction-wheel unloading.
    wheelMomentum=rw.A*(rw.J.*wheelSpeed);
    desiredTorque=-mtq.unloadGain*wheelMomentum;
    dipoleCommand=cross(fieldB,desiredTorque)/dot(fieldB,fieldB);
    [magneticTorque,dipoleCommand]=magnetorquerModel(dipoleCommand,fieldB,mtq);
    motorTorque=rw.A\magneticTorque;
    motorTorque=limitReactionWheelTorque(motorTorque,wheelSpeed,rw);
end
