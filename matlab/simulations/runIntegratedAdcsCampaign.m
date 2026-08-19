function results = runIntegratedAdcsCampaign(outputDir)
% RUNINTEGRATEDADCSCAMPAIGN Reproduce INT-001 end-to-end evidence.
    if nargin<1, outputDir=fullfile("artifacts","integration"); end
    root=fullfile(fileparts(mfilename("fullpath")),"..","..");
    for folder={"config","control","dynamics","environment","estimation","guidance","math","sensors","simulations"}
        addpath(fullfile(root,"matlab",folder{1}));
    end
    if ~exist(outputDir,"dir"), mkdir(outputDir); end
    names={"slew","inertial","nadir","desaturation","fault"};
    durations=[80,80,80,180,80]; results=cell(size(names));
    file=fopen(fullfile(outputDir,"integrated_results.csv"),"w");
    fprintf(file,"scenario,seed,success,final_pointing_deg,rms_rate_deg_s,settling_s,control_effort_nms,max_wheel_rad_s,final_estimator_deg,max_q_norm_error,saturation,initial_momentum_nms,final_momentum_nms\n");
    for k=1:numel(names)
        results{k}=simulateIntegratedAdcsScenario(names{k},3100+k,durations(k)); r=results{k};
        fprintf(file,"%s,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%.9g,%.9g\n", ...
            r.name,r.seed,r.success,rad2deg(r.pointingError(end)),rad2deg(sqrt(mean(r.rateError.^2))), ...
            r.settlingTime,r.controlEffort,max(r.maxWheelSpeed),rad2deg(r.estimatorError(end)), ...
            r.maxQuaternionNormError,r.saturationObserved,r.wheelMomentum(1),r.wheelMomentum(end));
        fprintf("%-14s %s point=%6.3f deg estimate=%6.3f deg qnorm=%.2e\n", ...
            r.name,passText(r.success),rad2deg(r.pointingError(end)),rad2deg(r.estimatorError(end)),r.maxQuaternionNormError);
    end
    fclose(file);
    assert(all(cellfun(@(r) r.success,results)),"Integrated campaign mission failure");
    assert(results{1}.saturationObserved,"Slew did not exercise actuator saturation");
    assert(results{4}.wheelMomentum(end)<results{4}.wheelMomentum(1),"Desaturation did not reduce momentum");
    assert(any(strcmp(results{5}.modes,"fault")),"Fault mode was not exercised");
    set(0,"defaultfigurevisible","off");
    figure; hold on; for k=1:3, r=results{k}; plot(r.times,rad2deg(r.pointingError),"DisplayName",r.name); end
    xlabel("Time [s]"); ylabel("Pointing error [deg]"); grid on; legend("show");
    print(fullfile(outputDir,"integrated_pointing.png"),"-dpng"); close;
    r=results{4}; figure; plot(r.times,r.wheelMomentum); grid on;
    xlabel("Time [s]"); ylabel("Wheel momentum [N m s]"); title("Integrated wheel desaturation");
    print(fullfile(outputDir,"integrated_desaturation.png"),"-dpng"); close;
    r=results{1}; figure; plot(r.times,rad2deg(r.estimatorError)); grid on;
    xlabel("Time [s]"); ylabel("Estimator attitude error [deg]"); title("Integrated noisy MEKF");
    print(fullfile(outputDir,"integrated_estimator.png"),"-dpng"); close;
    [~,revision]=system("git rev-parse HEAD"); file=fopen(fullfile(outputDir,"metadata.txt"),"w");
    fprintf(file,"git_revision=%s",revision); fprintf(file,"octave_version=%s\nseeds=3101:3105\nintegrator=fixed-step RK4\nstep_s=0.02\n",version); fclose(file);
end

function value=passText(pass)
    if pass, value="PASS"; else, value="FAIL"; end
end
