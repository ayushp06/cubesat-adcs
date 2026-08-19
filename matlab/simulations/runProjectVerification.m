function runProjectVerification(outputDir)
% RUNPROJECTVERIFICATION Generate the final reproducible verification evidence.
    if nargin<1, outputDir=fullfile("..","..","artifacts","verification"); end
    simulationDir=fileparts(mfilename("fullpath")); projectRoot=fullfile(simulationDir,"..","..");
    folders={"config","control","dynamics","environment","estimation","guidance","math","sensors","simulations"};
    for k=1:numel(folders), addpath(fullfile(projectRoot,"matlab",folders{k})); end
    if ~exist(outputDir,"dir"), mkdir(outputDir); end
    rows={};

    sc=spacecraftParams(); rw=reactionWheelParams(); options=odeset("RelTol",1e-9,"AbsTol",1e-10);
    x0=[1;0;0;0;deg2rad([2;3;4])]; [~,tf]=ode45(@(t,x) attitudeDynamics(t,x,sc,zeros(3,1)),linspace(0,100,501),x0,options);
    energy=.5*sum((tf(:,5:7)*sc.J).*tf(:,5:7),2);
    rows(end+1,:)={"torque_free","relative_energy_error",max(abs((energy-energy(1))/energy(1))),1e-7};

    [~,exchange]=ode45(@(t,x) attitudeDynamicsRW(t,x,sc,rw,1e-4),[0,10],[1;0;0;0;zeros(4,1)],options);
    total=sc.J(1,1)*exchange(:,5)+rw.J(1)*exchange(:,8);
    rows(end+1,:)={"wheel_exchange","momentum_error_nms",max(abs(total-total(1))),1e-12};

    axis=[1;2;3]/sqrt(14); slew=simulateAttitudeSlew([cos(pi/4);axis*sin(pi/4)],deg2rad([1;-.5;.75]),180);
    rows(end+1,:)={"arbitrary_slew","final_error_deg",slew.metrics.finalPointingErrorDeg,.01};

    earth=earthEnvironmentParams(); model=truthModelParams(); radius=earth.radius+400e3; speed=sqrt(earth.mu/radius);
    initial=[radius;0;0;0;speed;0;1;0;0;0;deg2rad([.1;-.05;.2])]; period=2*pi*sqrt(radius^3/earth.mu);
    [~,sixdof]=ode45(@(t,x) fullSpacecraftDynamics(t,x,sc,earth,model),linspace(0,period,501),initial,odeset("RelTol",1e-9,"AbsTol",1e-8));
    qError=max(abs(vecnorm(sixdof(:,7:10),2,2)-1)); rows(end+1,:)={"six_dof","quaternion_norm_error",qError,1e-6};

    mekf=simulateMekfScenario(21); rows(end+1,:)={"mekf","rms_attitude_error_deg",rad2deg(mekf.rmsAttitudeError),2};
    rows(end+1,:)={"mekf","bias_error_deg_s",rad2deg(mekf.finalBiasError),.08};
    detumble=simulateBdotDetumble(); rows(end+1,:)={"bdot","final_rate_deg_s",rad2deg(detumble.finalRate),1};
    nadir=runNadirPointingDemo(); rows(end+1,:)={"nadir","max_axis_error_rad",nadir.maxNadirError,1e-7};
    unload=simulateMomentumUnloading(); rows(end+1,:)={"desaturation","final_momentum_fraction",unload.finalFraction,.2};
    controllers=simulateControllerComparison();
    for k=1:3, rows(end+1,:)={lower(controllers(k).method),"settling_time_s",controllers(k).settlingTime,180}; end
    monteCarlo=runMonteCarloVerification(12); rows(end+1,:)={"monte_carlo","pass_fraction",monteCarlo.passFraction,1};
    fault=runFaultScenario(); rows(end+1,:)={"fault","transition_pass",double(fault.passed),1};
    [silStatus,~]=system(fullfile(projectRoot,"build","adcs_sil")); rows(end+1,:)={"sil","exit_status",silStatus,0};

    file=fopen(fullfile(outputDir,"verification_results.csv"),"w"); fprintf(file,"scenario,metric,value,limit,pass\n");
    for k=1:size(rows,1)
        if strcmp(rows{k,2},"pass_fraction") || strcmp(rows{k,2},"transition_pass"), pass=rows{k,3}>=rows{k,4}; else, pass=rows{k,3}<=rows{k,4}; end
        fprintf(file,"%s,%s,%.12g,%.12g,%d\n",rows{k,1},rows{k,2},rows{k,3},rows{k,4},pass);
    end
    fclose(file);
    [~,revision]=system("git rev-parse HEAD");
    file=fopen(fullfile(outputDir,"metadata.txt"),"w");
    fprintf(file,"git_revision=%s",revision);
    fprintf(file,"octave_version=%s\n",version);
    fprintf(file,"headline_mekf_seed=21\nmonte_carlo_seeds=1001:1012\n");
    fprintf(file,"campaign_orbit_reltol=1e-9\ncampaign_orbit_abstol=1e-8\n");
    fprintf(file,"configuration=repository MATLAB parameter functions at git_revision\n");
    fclose(file);
    file=fopen(fullfile(outputDir,"monte_carlo.csv"),"w"); fprintf(file,"seed,attitude_rms_deg,bias_error_deg_s\n");
    for k=1:numel(monteCarlo.seeds), fprintf(file,"%d,%.8f,%.8f\n",monteCarlo.seeds(k),monteCarlo.attitudeRmsDeg(k),monteCarlo.biasErrorDegPerSec(k)); end
    fclose(file);

    set(0,"defaultfigurevisible","off");
    plotSeries(slew.t,attitudeErrors(slew),"Time [s]","Error [deg]","Arbitrary-axis slew",fullfile(outputDir,"slew_error.png"));
    plotSeries(mekf.times,rad2deg(mekf.attitudeError),"Time [s]","Error [deg]","Noisy MEKF with dropouts",fullfile(outputDir,"mekf_error.png"));
    plotSeries(detumble.times,rad2deg(detumble.rates),"Time [s]","Rate [deg/s]","B-dot detumble",fullfile(outputDir,"bdot_detumble.png"));
    plotSeries(unload.time,unload.momentum,"Time [s]","Wheel momentum [N m s]","Momentum unloading",fullfile(outputDir,"wheel_desaturation.png"));
    figure; bar(monteCarlo.attitudeRmsDeg); xlabel("Run"); ylabel("RMS error [deg]"); title("MEKF Monte Carlo"); grid on; print(fullfile(outputDir,"monte_carlo.png"),"-dpng"); close;
    fprintf("PROJECT VERIFICATION COMPLETE: %d checks, Monte Carlo pass %.0f%%\n",size(rows,1),100*monteCarlo.passFraction);
end

function errors=attitudeErrors(result)
    errors=zeros(numel(result.t),1);
    for k=1:numel(errors)
        q=quaternionAttitudeError(result.qReference,result.x(k,1:4)');
        errors(k)=rad2deg(2*acos(max(-1,min(1,q(1)))));
    end
end

function plotSeries(x,y,xLabel,yLabel,plotTitle,fileName)
    figure; plot(x,y,"LineWidth",1.2); xlabel(xLabel); ylabel(yLabel); title(plotTitle); grid on;
    print(fileName,"-dpng"); close;
end
