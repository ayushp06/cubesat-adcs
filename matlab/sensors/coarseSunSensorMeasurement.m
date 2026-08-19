function [sunDirectionB, valid, currents] = coarseSunSensorMeasurement(sunDirectionBTruth, illumination, params)
% COARSESUNSENSORMEASUREMENT Six face sensors with cosine response and dropout.
    normals = [eye(3), -eye(3)];
    currents = illumination * max(0, normals' * sunDirectionBTruth) + ...
        params.noiseStd * randn(6,1);
    currents = max(0, currents);
    active = currents >= params.detectionThreshold;
    valid = illumination > 0 && any(active);
    if valid
        sunDirectionB = normals(:,active) * currents(active);
        sunDirectionB = sunDirectionB / norm(sunDirectionB);
    else
        sunDirectionB = [NaN; NaN; NaN];
    end
end
