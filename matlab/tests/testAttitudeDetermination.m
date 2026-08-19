clear; clc;
testDir=fileparts(mfilename("fullpath"));
addpath(fullfile(testDir,"..","estimation")); addpath(fullfile(testDir,"..","math"));
qTruth=[cos(pi/6);0;0;sin(pi/6)]; dcm=quatToDCM(qTruth);
references=[1,0,1;0,1,1;0,0,1]; references=references./vecnorm(references);
observations=dcm'*references;
qTriad=triadAttitude(observations(:,1:2),references(:,1:2));
qQuest=questAttitude(observations,references,[1,2,0.5]);
assert(2*acos(min(1,abs(dot(qTriad,qTruth)))) < 1e-12);
assert(2*acos(min(1,abs(dot(qQuest,qTruth)))) < 1e-12);
assert(norm(quatToDCM(dcmToQuat(dcm))-dcm,"fro") < 1e-12);
failed=false;
try, triadAttitude([1,2;0,0;0,0],[1,2;0,0;0,0]); catch, failed=true; end
assert(failed);
disp("ALL ATTITUDE DETERMINATION TESTS PASSED");
