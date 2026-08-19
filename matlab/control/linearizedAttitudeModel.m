function [A,B] = linearizedAttitudeModel(inertia)
% LINEARIZEDATTITUDEMODEL x=[delta-theta; omega], torque input near rest.
    A=[zeros(3),eye(3);zeros(3),zeros(3)];
    B=[zeros(3);inertia\eye(3)];
end
