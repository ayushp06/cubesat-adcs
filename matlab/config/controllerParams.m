function controller = controllerParams(sc)
% CONTROLLERPARAMS Quaternion PD gains for the nominal spacecraft.

    naturalFrequency = 0.12;
    dampingRatio = 0.9;
    principalInertia = diag(sc.J);

    controller.Kp = diag(2 * principalInertia * naturalFrequency^2);
    controller.Kd = diag(2 * dampingRatio * principalInertia * naturalFrequency);
    controller.Ki = diag(0.02 * principalInertia * naturalFrequency^3);
    controller.integralLimit = deg2rad(20) * ones(3,1);
    controller.pointingTolerance = deg2rad(1);
    controller.rateTolerance = deg2rad(0.1);

end
