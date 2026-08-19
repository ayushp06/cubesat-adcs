function measurement = magnetometerMeasurement(fieldB, params)
% MAGNETOMETERMEASUREMENT Three-axis field measurement [T].
    measurement = (eye(3) + diag(params.scaleFactor)) * ...
        (eye(3) + params.misalignment) * fieldB + params.bias + ...
        params.noiseStd * randn(3,1);
    measurement = max(-params.maxField, min(params.maxField, measurement));
end
