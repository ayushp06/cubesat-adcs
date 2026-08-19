function qReference = inertialPointing(qCommand)
% INERTIALPOINTING Hold a commanded body-to-inertial attitude.
    qReference = quatNormalize(qCommand);
end
