function qError = quaternionAttitudeError(qReference, q)
% QUATERNIONATTITUDEERROR Shortest rotation from reference to current attitude.
% qReference and q are scalar-first Hamilton body-to-inertial quaternions.

    qError = quatMultiply( ...
        quatConjugate(quatNormalize(qReference)), ...
        quatNormalize(q));

    if qError(1) < 0
        qError = -qError;
    end

end
