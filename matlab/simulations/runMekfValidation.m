clear; clc;
simulationDir=fileparts(mfilename("fullpath"));
addpath(fullfile(simulationDir,"..","config"),fullfile(simulationDir,"..","sensors"), ...
    fullfile(simulationDir,"..","estimation"),fullfile(simulationDir,"..","math"));
results=simulateMekfScenario(21);
fprintf("Final attitude error: %.4f deg\n",rad2deg(results.attitudeError(end)));
fprintf("Last-10-s RMS attitude error: %.4f deg\n",rad2deg(results.rmsAttitudeError));
fprintf("Final gyro-bias error: %.5f deg/s\n",rad2deg(results.finalBiasError));
fprintf("First time below 2 deg: %.2f s\n",results.convergenceTime);
fprintf("Maximum vector-update NIS: %.3f\n",max(results.nis));
fprintf("Minimum covariance eigenvalue: %.3e\n",results.minimumCovarianceEigenvalue);
