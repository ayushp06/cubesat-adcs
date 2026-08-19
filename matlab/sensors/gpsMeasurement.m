function [positionECI, velocityECI, valid] = gpsMeasurement(positionTruth, velocityTruth, params)
% GPSMEASUREMENT Sampled ECI position/velocity with white noise and dropout.
    valid = rand() >= params.dropoutProbability;
    if valid
        positionECI = positionTruth + params.positionNoiseStd * randn(3,1);
        velocityECI = velocityTruth + params.velocityNoiseStd * randn(3,1);
    else
        positionECI = NaN(3,1); velocityECI = NaN(3,1);
    end
end
