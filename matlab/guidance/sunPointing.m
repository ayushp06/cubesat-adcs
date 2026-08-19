function qReference = sunPointing(sunDirectionECI)
% SUNPOINTING Point +Z_B at Sun while keeping +X_B near inertial +Z.
    secondary=[0;0;1];
    if norm(cross(sunDirectionECI,secondary))<1e-8, secondary=[1;0;0]; end
    qReference=vectorPointingAttitude([0;0;1],sunDirectionECI,[1;0;0],secondary);
end
