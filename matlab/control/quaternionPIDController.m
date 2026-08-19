function [bodyTorque,integralError] = quaternionPIDController(qReference,q,omega,integralError,dt,controller,maxTorque)
% QUATERNIONPIDCONTROLLER Bounded integral augmentation of quaternion PD.
    qError=quaternionAttitudeError(qReference,q);
    angleError=2*qError(2:4);
    candidate=max(-controller.integralLimit,min(controller.integralLimit,integralError+dt*angleError));
    unsaturated=-0.5*controller.Kp*angleError-controller.Kd*omega-controller.Ki*candidate;
    bodyTorque=max(-maxTorque,min(maxTorque,unsaturated));
    if all(abs(unsaturated)<=maxTorque), integralError=candidate; end
end
