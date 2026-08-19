clear; clc;
testDir=fileparts(mfilename("fullpath"));
addpath(fullfile(testDir,"..","config"),fullfile(testDir,"..","control"), ...
    fullfile(testDir,"..","dynamics"),fullfile(testDir,"..","math"),fullfile(testDir,"..","simulations"));
sc=spacecraftParams(); rw=reactionWheelParams(); pd=controllerParams(sc); lqr=lqrControllerParams(sc);
assert(lqr.controllabilityRank==6 && all(real(lqr.closedLoopEigenvalues)<0));
assert(norm(lqr.P-lqr.P','fro')<1e-10 && min(eig(lqr.P))>0);
[torque,integral]=quaternionPIDController([1;0;0;0],[0;1;0;0],zeros(3,1),zeros(3,1),1,pd,rw.maxTorque);
assert(all(abs(torque)<=rw.maxTorque) && all(abs(integral)<=pd.integralLimit));
results=simulateControllerComparison();
for k=1:3
    assert(results(k).finalErrorDeg<0.1 && ~isnan(results(k).settlingTime));
    assert(results(k).peakTorque<=norm(rw.maxTorque)+1e-12);
end
disp("ALL ADVANCED CONTROLLER TESTS PASSED");
