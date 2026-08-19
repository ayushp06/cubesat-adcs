function bodyTorque = quaternionPDController( ...
    qReference, q, omega, controller)
% QUATERNIONPDCONTROLLER Command body torque from attitude and rate error.

    qError = quaternionAttitudeError(qReference, q);
    bodyTorque = ...
        -controller.Kp * qError(2:4) ...
        -controller.Kd * omega;

end
