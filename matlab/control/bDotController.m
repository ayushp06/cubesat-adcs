function dipoleCommandB = bDotController(fieldB,previousFieldB,dt,mtq)
% BDOTCONTROLLER Command magnetic dipole opposing measured field derivative.
    dipoleCommandB=-mtq.bDotGain*(fieldB-previousFieldB)/dt;
end
