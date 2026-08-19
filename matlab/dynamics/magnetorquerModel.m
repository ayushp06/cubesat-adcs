function [torqueB,appliedDipoleB] = magnetorquerModel(commandedDipoleB,fieldB,mtq)
% MAGNETORQUERMODEL Allocate, saturate, and apply three magnetic rods.
    rodDipoles=mtq.axes\commandedDipoleB;
    rodDipoles=max(-mtq.maxDipole,min(mtq.maxDipole,rodDipoles));
    appliedDipoleB=mtq.axes*rodDipoles;
    torqueB=cross(appliedDipoleB,fieldB);
end
