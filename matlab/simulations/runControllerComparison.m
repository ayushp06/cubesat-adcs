clear; clc;
simulationDir=fileparts(mfilename("fullpath"));
addpath(fullfile(simulationDir,"..","config"),fullfile(simulationDir,"..","control"), ...
    fullfile(simulationDir,"..","dynamics"),fullfile(simulationDir,"..","math"));
results=simulateControllerComparison();
for k=1:numel(results)
    fprintf("%s: final %.4f deg, settle %.2f s, peak rate %.3f deg/s, effort %.4e Nms\n", ...
        results(k).method,results(k).finalErrorDeg,results(k).settlingTime, ...
        results(k).peakRateDegPerSec,results(k).controlEffort);
end
