function qReference = safeAttitude(sunDirectionECI)
% SAFEATTITUDE Power-positive passive-safe target: +Z_B toward the Sun.
    qReference=sunPointing(sunDirectionECI);
end
