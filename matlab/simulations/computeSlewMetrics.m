function metrics = computeSlewMetrics(result, rw, controller)
% COMPUTESLEWMETRICS Summarize pointing and reaction-wheel performance.

    t = result.t;
    x = result.x;
    qReference = result.qReference;
    qInitial = quatNormalize(x(1,1:4)');
    sampleCount = size(x,1);
    pointingError = zeros(sampleCount, 1);
    progress = zeros(sampleCount, 1);

    targetDelta = quaternionAttitudeError(qReference, qInitial);
    targetAngle = 2 * acos(max(-1, min(1, targetDelta(1))));
    targetAxis = zeros(3,1);
    if targetAngle > eps
        targetAxis = -targetDelta(2:4) / sin(targetAngle / 2);
    end

    for k = 1:sampleCount
        q = quatNormalize(x(k,1:4)');
        qError = quaternionAttitudeError(qReference, q);
        pointingError(k) = 2 * acos(max(-1, min(1, qError(1))));

        qFromInitial = quatMultiply(quatConjugate(qInitial), q);
        if qFromInitial(1) < 0
            qFromInitial = -qFromInitial;
        end
        angle = 2 * acos(max(-1, min(1, qFromInitial(1))));
        if angle > eps
            axis = qFromInitial(2:4) / sin(angle / 2);
            progress(k) = angle * dot(axis, targetAxis);
        end
    end

    bodyRate = vecnorm(x(:,5:7), 2, 2);
    settled = pointingError <= controller.pointingTolerance & ...
        bodyRate <= controller.rateTolerance;
    lastOutside = find(~settled, 1, "last");

    if isempty(lastOutside)
        settlingTime = t(1);
    elseif lastOutside < sampleCount
        settlingTime = t(lastOutside + 1);
    else
        settlingTime = NaN;
    end

    wheelMomentum = x(:,8:10) .* rw.J';
    totalWheelMomentum = wheelMomentum * rw.A';

    metrics.finalPointingErrorDeg = rad2deg(pointingError(end));
    metrics.settlingTimeSec = settlingTime;
    metrics.overshootDeg = rad2deg(max([0; progress - targetAngle]));
    metrics.peakBodyRateDegPerSec = rad2deg(max(bodyRate));
    metrics.peakBodyRateByAxisDegPerSec = rad2deg(max(abs(x(:,5:7)), [], 1));
    metrics.controlEffortNms = trapz(t, abs(result.appliedMotorTorque), 1);
    metrics.peakWheelSpeedRadPerSec = max(abs(x(:,8:10)), [], 1);
    metrics.peakWheelMomentumNms = max(abs(wheelMomentum), [], 1);
    metrics.peakTotalWheelMomentumNms = ...
        max(vecnorm(totalWheelMomentum, 2, 2));

end
