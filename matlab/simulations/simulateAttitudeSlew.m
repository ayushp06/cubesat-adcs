function result = simulateAttitudeSlew(qReference, omega0, duration, options)
% SIMULATEATTITUDESLEW Run a closed-loop three-wheel attitude maneuver.

    if nargin < 4
        options = odeset("RelTol", 1e-8, "AbsTol", 1e-10);
    end

    sc = spacecraftParams();
    rw = reactionWheelParams();
    controller = controllerParams(sc);
    qReference = quatNormalize(qReference);
    x0 = [1; 0; 0; 0; omega0; zeros(3,1)];
    outputTimes = linspace(0, duration, round(duration * 10) + 1);

    [t, x] = ode45( ...
        @(time, state) closedLoopAttitudeDynamics( ...
            time, state, sc, rw, controller, qReference), ...
        outputTimes, x0, options);

    appliedMotorTorque = zeros(size(x,1), 3);

    for k = 1:size(x,1)
        bodyTorque = quaternionPDController( ...
            qReference, x(k,1:4)', x(k,5:7)', controller);
        command = allocateReactionWheelTorque(bodyTorque, rw);
        appliedMotorTorque(k,:) = limitReactionWheelTorque( ...
            command, x(k,8:10)', rw)';
    end

    result.t = t;
    result.x = x;
    result.qReference = qReference;
    result.appliedMotorTorque = appliedMotorTorque;
    result.metrics = computeSlewMetrics(result, rw, controller);

end
