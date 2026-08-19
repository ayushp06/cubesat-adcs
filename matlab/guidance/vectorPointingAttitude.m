function qReference = vectorPointingAttitude(bodyAxis, targetECI, bodySecondary, secondaryECI)
% VECTORPOINTINGATTITUDE Align a body axis and resolve roll with a second vector.
    qReference = triadAttitude([bodyAxis,bodySecondary],[targetECI,secondaryECI]);
end
