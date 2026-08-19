function results = simulateMekfScenario(seed)
% SIMULATEMEKFSCENARIO Reproducible noisy attitude/bias estimation with outages.
    if nargin < 1, seed = 21; end
    rng(seed);
    sensors=sensorParams(); filter=mekfParams(); dt=sensors.gyro.samplePeriod;
    times=0:dt:60; count=numel(times);
    qTruth=[1;0;0;0]; omegaTruth=deg2rad([0.15;-0.1;0.2]);
    biasTruth=deg2rad([0.3;-0.2;0.15]);
    qInitial=[cos(deg2rad(20)/2);sin(deg2rad(20)/2);0;0];
    estimate=mekfInitialize(qInitial,zeros(3,1),filter);
    magneticECI=[0.3;-0.1;0.95]; magneticECI=magneticECI/norm(magneticECI);
    sunECI=[1;0.2;0.1]; sunECI=sunECI/norm(sunECI);
    attitudeError=zeros(count,1); biasError=zeros(count,1); covarianceTrace=zeros(count,1);
    nis=[]; minimumEigenvalue=Inf;

    for index=1:count
        if index>1
            angle=norm(omegaTruth)*dt;
            qTruth=quatNormalize(quatMultiply(qTruth,[cos(angle/2);sin(angle/2)*omegaTruth/norm(omegaTruth)]));
        end
        [gyro,biasTruth]=gyroMeasurement(omegaTruth,biasTruth,dt,sensors.gyro);
        estimate=mekfPredict(estimate,gyro,dt,filter);
        time=times(index); dcmBI=quatToDCM(qTruth)';

        if mod(index-1,round(sensors.magnetometer.samplePeriod/dt))==0 && ~(time>=25 && time<32)
            fieldMagnitude=45e-6;
            mag=magnetometerMeasurement(fieldMagnitude*dcmBI*magneticECI,sensors.magnetometer);
            estimate=mekfUpdateVectors(estimate,mag,magneticECI,filter.magnetometerVectorNoiseStd);
            nis(end+1)=estimate.lastNIS;
        end
        if mod(index-1,round(sensors.sun.samplePeriod/dt))==0
            illumination=double(~(time>=20 && time<35));
            [sun,valid]=coarseSunSensorMeasurement(dcmBI*sunECI,illumination,sensors.sun);
            if valid
                estimate=mekfUpdateVectors(estimate,sun,sunECI,filter.sunVectorNoiseStd);
                nis(end+1)=estimate.lastNIS;
            end
        end

        attitudeError(index)=2*acos(min(1,abs(dot(estimate.qIB,qTruth))));
        biasError(index)=norm(estimate.gyroBias-biasTruth);
        covarianceTrace(index)=trace(estimate.covariance);
        minimumEigenvalue=min(minimumEigenvalue,min(eig(estimate.covariance)));
    end
    tail=times>=50;
    results.times=times; results.attitudeError=attitudeError;
    results.biasError=biasError; results.covarianceTrace=covarianceTrace;
    results.nis=nis; results.rmsAttitudeError=sqrt(mean(attitudeError(tail).^2));
    results.finalBiasError=biasError(end); results.minimumCovarianceEigenvalue=minimumEigenvalue;
    converged=find(attitudeError<deg2rad(2),1);
    if isempty(converged), results.convergenceTime=Inf; else, results.convergenceTime=times(converged); end
    results.finalEstimate=estimate;
end
