clear; clc;
testDir=fileparts(mfilename("fullpath"));
addpath(fullfile(testDir,"..","guidance"),fullfile(testDir,"..","estimation"),fullfile(testDir,"..","math"));

q=[cos(0.3);sin(0.3);0;0]; assert(norm(inertialPointing(2*q)-q)<1e-12);
sun=[1;2;3]/sqrt(14); qSun=sunPointing(sun); C=quatToDCM(qSun);
assert(norm(C*[0;0;1]-sun)<1e-12);
assert(norm(quatToDCM(safeAttitude(sun))*[0;0;1]-sun)<1e-12);

r=[7e6;0;0]; v=[0;7500;0]; qNadir=nadirPointing(r,v); C=quatToDCM(qNadir);
assert(norm(C*[0;0;1]+r/norm(r))<1e-12);
assert(norm(C*[1;0;0]-v/norm(v))<1e-12 && abs(det(C)-1)<1e-12);

q0=[1;0;0;0]; qf=[cos(pi/4);0;0;sin(pi/4)];
[qa,wa]=quaternionSlewReference(q0,qf,0,10); [qm,wm]=quaternionSlewReference(q0,qf,5,10);
[qb,wb]=quaternionSlewReference(q0,qf,10,10);
assert(norm(qa-q0)<1e-12 && norm(qb-qf)<1e-12 && norm(wa)==0 && norm(wb)<1e-12);
assert(abs(2*acos(abs(dot(qm,q0)))-pi/4)<1e-12 && abs(wm(3)-3*pi/40)<1e-12);
disp("ALL GUIDANCE TESTS PASSED");
