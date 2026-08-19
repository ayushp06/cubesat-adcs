function bodyTorque = quaternionLqrController(qReference,q,omega,controller)
% QUATERNIONLQRCONTROLLER Apply small-angle LQR gain to nonlinear error state.
    qError=quaternionAttitudeError(qReference,q);
    bodyTorque=-controller.gain*[2*qError(2:4);omega];
end
