function [qMeasured, valid] = starTrackerMeasurement(qTruth, params)
% STARTRACKERMEASUREMENT Optional quaternion observation with dropout.
    valid = params.enabled && rand() >= params.dropoutProbability;
    if ~valid, qMeasured = NaN(4,1); return; end
    rotationVector = params.angleNoiseStd * randn(3,1);
    angle = norm(rotationVector);
    if angle == 0
        errorQuaternion = [1;0;0;0];
    else
        errorQuaternion = [cos(angle/2); sin(angle/2)*rotationVector/angle];
    end
    qMeasured = quatNormalize(quatMultiply(qTruth, errorQuaternion));
end
