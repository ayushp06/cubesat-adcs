clear; clc;
testDir=fileparts(mfilename("fullpath"));
for folder={"config","control","dynamics","environment","estimation","guidance","math","sensors","simulations"}
    addpath(fullfile(testDir,"..",folder{1}));
end
assert(nargin("flightAdcsStep")==5); % Interface cannot accept truth attitude/rate.
result=simulateIntegratedAdcsScenario("slew",3101,80);
assert(result.success);
assert(result.saturationObserved);
assert(rad2deg(result.pointingError(end))<3);
assert(rad2deg(result.estimatorError(end))<3);
assert(result.maxQuaternionNormError<1e-10);
assert(all(result.maxWheelSpeed<=reactionWheelParams().maxSpeed'));
assert(all(result.sensorUpdates>0));
assert(any(strcmp(result.modes,"slew")) && strcmp(result.modes{end},"nominal"));
repeat=simulateIntegratedAdcsScenario("slew",3101,80);
assert(isequal(result.state,repeat.state));
disp("ALL INTEGRATED ADCS TESTS PASSED");
