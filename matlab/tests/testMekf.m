clear; clc;
testDir=fileparts(mfilename("fullpath"));
addpath(fullfile(testDir,"..","config"),fullfile(testDir,"..","sensors"), ...
    fullfile(testDir,"..","estimation"),fullfile(testDir,"..","math"), ...
    fullfile(testDir,"..","simulations"));

p=mekfParams(); p.gyroNoiseStd=0; p.biasRandomWalkStd=0;
estimate=mekfInitialize([1;0;0;0],zeros(3,1),p);
estimate=mekfPredict(estimate,[0;0;0.1],1,p);
assert(abs(2*acos(estimate.qIB(1))-0.1)<1e-12);
truth=[cos(pi/18);0;0;sin(pi/18)]; refs=[1,0;0,1;0,0]; body=quatToDCM(truth)'*refs;
for index=1:8, estimate=mekfUpdateVectors(estimate,body,refs,1e-3); end
assert(2*acos(min(1,abs(dot(estimate.qIB,truth))))<deg2rad(0.1));

results=simulateMekfScenario(21);
assert(rad2deg(results.rmsAttitudeError)<2);
assert(rad2deg(results.finalBiasError)<0.08);
assert(results.minimumCovarianceEigenvalue>-1e-12);
assert(all(isfinite(results.nis)) && all(results.nis>=0));
disp("ALL MEKF TESTS PASSED");
