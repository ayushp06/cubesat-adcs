function results=runMonteCarloVerification(runCount)
% RUNMONTECARLOVERIFICATION Seeded MEKF noise/dropout campaign.
    if nargin<1, runCount=12; end
    attitude=zeros(runCount,1); bias=zeros(runCount,1);
    for k=1:runCount
        scenario=simulateMekfScenario(1000+k);
        attitude(k)=rad2deg(scenario.rmsAttitudeError);
        bias(k)=rad2deg(scenario.finalBiasError);
    end
    results.seeds=(1001:1000+runCount)'; results.attitudeRmsDeg=attitude;
    results.biasErrorDegPerSec=bias; results.passFraction=mean(attitude<2 & bias<0.08);
end
