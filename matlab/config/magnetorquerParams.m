function mtq = magnetorquerParams()
% MAGNETORQUERPARAMS Orthogonal three-rod magnetic actuator assumptions.
    mtq.axes=eye(3);
    mtq.maxDipole=0.2*ones(3,1); % [A m^2] per rod
    mtq.bDotGain=2e4;            % [A m^2/(T/s)]
    mtq.unloadGain=2e-3;         % [1/s]
end
