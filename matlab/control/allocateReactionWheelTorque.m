function motorTorque = allocateReactionWheelTorque(bodyTorque, rw)
% ALLOCATEREACTIONWHEELTORQUE Map desired body torque to three wheel torques.

    motorTorque = rw.A \ (-bodyTorque);

end
