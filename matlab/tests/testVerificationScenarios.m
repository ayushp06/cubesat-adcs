clear; clc;
testDir=fileparts(mfilename("fullpath")); addpath(genpath(fullfile(testDir,"..")));
nadir=runNadirPointingDemo(); fault=runFaultScenario(); monteCarlo=runMonteCarloVerification(2);
assert(nadir.maxNadirError<1e-7 && nadir.maxOrthogonalityError<1e-12);
assert(fault.passed && monteCarlo.passFraction==1);
disp("ALL VERIFICATION SCENARIO TESTS PASSED");
