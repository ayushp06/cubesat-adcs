function [qReference,omegaReferenceB] = quaternionSlewReference(qStart,qEnd,time,duration)
% QUATERNIONSLEWREFERENCE Shortest cubic-time SLERP and feed-forward body rate.
    qStart=quatNormalize(qStart); qEnd=quatNormalize(qEnd);
    if dot(qStart,qEnd)<0, qEnd=-qEnd; end
    relative=quatMultiply(quatConjugate(qStart),qEnd);
    angle=2*acos(max(-1,min(1,relative(1))));
    if angle<1e-12, qReference=qStart; omegaReferenceB=zeros(3,1); return; end
    axis=relative(2:4)/sin(angle/2);
    u=max(0,min(1,time/duration)); s=3*u^2-2*u^3;
    rate=(6*u-6*u^2)/duration;
    qReference=quatNormalize(quatMultiply(qStart,[cos(s*angle/2);axis*sin(s*angle/2)]));
    omegaReferenceB=axis*angle*rate;
end
