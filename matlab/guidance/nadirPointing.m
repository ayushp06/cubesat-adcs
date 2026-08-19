function qReference = nadirPointing(positionECI,velocityECI)
% NADIRPOINTING LVLH attitude: +Z nadir, +X along-track, +Y completes frame.
    zAxis=-positionECI/norm(positionECI);
    alongTrack=velocityECI-dot(velocityECI,zAxis)*zAxis;
    xAxis=alongTrack/norm(alongTrack);
    yAxis=cross(zAxis,xAxis);
    qReference=dcmToQuat([xAxis,yAxis,zAxis]);
end
