function xdot = closedLoopAttitudeDynamics( ...
    t, x, sc, rw, controller, qReference)
% CLOSEDLOOPATTITUDEDYNAMICS Quaternion PD control through reaction wheels.

    bodyTorque = quaternionPDController( ...
        qReference, x(1:4), x(5:7), controller);
    motorTorque = allocateReactionWheelTorque(bodyTorque, rw);
    xdot = attitudeDynamics3RW(t, x, sc, rw, motorTorque);

end
