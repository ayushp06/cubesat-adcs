function result = simulateIntegratedAdcsScenario(name,seed,duration)
% SIMULATEINTEGRATEDADCSSCENARIO Seeded full truth/sensor/MEKF/control loop.
    if nargin<1, name="slew"; end
    if nargin<2, seed=3101; end
    if nargin<3, duration=120; end
    rng(seed,"twister"); dt=.02; steps=round(duration/dt)+1; times=(0:steps-1)'*dt;
    sc=spacecraftParams(); rw=reactionWheelParams(); mtq=magnetorquerParams();
    earth=earthEnvironmentParams(); model=truthModelParams(); sensors=sensorParams();
    configuration=struct("rw",rw,"mtq",mtq,"mekf",mekfParams(), ...
        "mode",modeManagerParams(),"controller",controllerParams(sc));
    radius=earth.radius+500e3; speed=sqrt(earth.mu/radius);
    axis=[1;2;3]/sqrt(14); qTarget=[cos(deg2rad(35));axis*sin(deg2rad(35))];
    q0=[1;0;0;0]; wheel0=zeros(3,1);
    if strcmp(name,"desaturation")
        q0=safeAttitude(sunVectorECI(0,earth));
        wheel0=.9*rw.maxSpeed.*[1;-.8;.7]; qTarget=q0;
    end
    x=[radius;0;0;0;speed;0;q0;deg2rad([.05;-.03;.04]);wheel0];
    initialEstimate=quatMultiply(q0,[cos(deg2rad(5));sin(deg2rad(5));0;0]);
    flight.estimate=mekfInitialize(initialEstimate,zeros(3,1),configuration.mekf);
    flight.mode=initializeAdcsMode(); flight.lastReference=q0;
    flight.previousMag=zeros(3,1); flight.previousMagValid=false;
    gyroBias=deg2rad([.08;-.05;.04]);
    motor=zeros(3,1); dipole=zeros(3,1);
    request.qStart=q0; request.qTarget=qTarget; request.slewStart=2;
    request.slewDuration=40; request.guidance="inertial";
    if strcmp(name,"nadir"), request.guidance="nadir"; end
    request.status=blankStatus();

    state=zeros(steps,16); estimate=zeros(steps,4); bias=zeros(steps,3);
    reference=zeros(steps,4); rateReference=zeros(steps,3); appliedMotor=zeros(steps,3);
    modes=cell(steps,1); magUpdates=0; sunUpdates=0; gpsUpdates=0;
    saturation=false(steps,1); qNormError=zeros(steps,1);
    gpsPosition=x(1:3); gpsVelocity=x(4:6); gpsValid=true;
    for k=1:steps
        time=times(k); q=x(7:10); rotation=quatToDCM(q);
        fieldECI=earthMagneticField(x(1:3),time,earth); sunECI=sunVectorECI(time,earth);
        illumination=eclipseFactor(x(1:3),sunECI,earth);
        [gyro,gyroBias]=gyroMeasurement(x(11:13),gyroBias,dt,sensors.gyro);
        magTick=mod(k-1,round(sensors.magnetometer.samplePeriod/dt))==0;
        sunTick=mod(k-1,round(sensors.sun.samplePeriod/dt))==0;
        gpsTick=mod(k-1,round(sensors.gps.samplePeriod/dt))==0;
        forcedDropout=strcmp(name,"fault") && time>=35 && time<45;
        magValid=magTick && ~forcedDropout; sunValid=false;
        magBody=zeros(3,1); sunBody=zeros(3,1);
        if magValid, magBody=magnetometerMeasurement(rotation'*fieldECI,sensors.magnetometer); magUpdates=magUpdates+1; end
        if sunTick && ~forcedDropout
            [sunBody,sunValid]=coarseSunSensorMeasurement(rotation'*sunECI,illumination,sensors.sun);
            sunUpdates=sunUpdates+sunValid;
        end
        if gpsTick
            [gpsPosition,gpsVelocity,gpsValid]=gpsMeasurement(x(1:3),x(4:6),sensors.gps);
            if forcedDropout, gpsValid=false; end
            gpsUpdates=gpsUpdates+gpsValid;
        end
        packet=struct("dt",dt,"gyro",gyro,"magBody",magBody,"magECI",fieldECI, ...
            "magValid",magValid,"sunBody",sunBody,"sunECI",sunECI,"sunValid",sunValid, ...
            "positionECI",gpsPosition,"velocityECI",gpsVelocity,"gpsValid",gpsValid, ...
            "wheelSpeed",x(14:16),"magPeriod",sensors.magnetometer.samplePeriod);
        request.status=modeStatus(name,time,request,x,rw,forcedDropout);
        [flight,command]=flightAdcsStep(time,flight,packet,request,configuration);
        motor=command.motorTorque; dipole=command.dipole;
        state(k,:)=x'; estimate(k,:)=flight.estimate.qIB'; bias(k,:)=flight.estimate.gyroBias';
        reference(k,:)=command.qReference'; rateReference(k,:)=command.omegaReference';
        appliedMotor(k,:)=motor'; modes{k}=char(command.mode);
        saturation(k)=any(abs(motor)>=rw.maxTorque-10*eps);
        qNormError(k)=abs(norm(q)-1);
        if k<steps
            f=@(localTime,localState) integratedSpacecraftDynamics(localTime,localState,sc,rw,mtq,earth,model,motor,dipole);
            k1=f(time,x); k2=f(time+dt/2,x+dt*k1/2); k3=f(time+dt/2,x+dt*k2/2); k4=f(time+dt,x+dt*k3);
            x=x+dt*(k1+2*k2+2*k3+k4)/6; x(7:10)=quatNormalize(x(7:10));
        end
    end
    pointing=zeros(steps,1); estimatorError=zeros(steps,1);
    for k=1:steps
        pointing(k)=angleError(reference(k,:)',state(k,7:10)');
        estimatorError(k)=angleError(estimate(k,:)',state(k,7:10)');
    end
    rateError=vecnorm(state(:,11:13)-rateReference,2,2);
    momentum=vecnorm((rw.A*(rw.J.*state(:,14:16)'))',2,2);
    settled=find(pointing<deg2rad(2) & rateError<deg2rad(.2),1);
    if isempty(settled), settlingTime=Inf; else, settlingTime=times(settled); end
    result=struct("name",char(name),"seed",seed,"times",times,"state",state, ...
        "estimate",estimate,"reference",reference,"modes",{modes}, ...
        "pointingError",pointing,"rateError",rateError,"estimatorError",estimatorError, ...
        "settlingTime",settlingTime,"controlEffort",sum(vecnorm(appliedMotor,2,2))*dt, ...
        "wheelMomentum",momentum,"maxWheelSpeed",max(abs(state(:,14:16)),[],1), ...
        "maxQuaternionNormError",max(qNormError),"saturationObserved",any(saturation), ...
        "sensorUpdates",[magUpdates,sunUpdates,gpsUpdates]);
    result.success=missionSuccess(result,name,rw);
end

function status=blankStatus()
    status=struct("fault",false,"faultReset",false,"initializationComplete",false, ...
        "bodyRate",0,"wheelSpeedFraction",0,"safeRequested",false, ...
        "estimatorValid",true,"slewRequested",false,"slewComplete",false,"nominalRequested",false);
end

function status=modeStatus(name,time,request,x,rw,dropout)
    status=blankStatus(); status.initializationComplete=time>=.1;
    status.estimatorValid=all(isfinite(x)); status.bodyRate=0;
    status.wheelSpeedFraction=max(abs(x(14:16))./rw.maxSpeed);
    status.slewRequested=strcmp(name,"slew") && time>=request.slewStart && time<request.slewStart+request.slewDuration;
    status.slewComplete=time>=request.slewStart+request.slewDuration;
    status.nominalRequested=~strcmp(name,"desaturation");
    status.fault=strcmp(name,"fault") && dropout && time>=40;
    status.faultReset=strcmp(name,"fault") && time>=45 && time<45.1;
end

function angle=angleError(qA,qB)
    error=quatMultiply(quatConjugate(qA),qB);
    angle=2*acos(max(-1,min(1,abs(error(1)))));
end

function success=missionSuccess(result,name,rw)
    finite=all(isfinite(result.state(:))) && all(isfinite(result.estimate(:)));
    bounded=result.maxQuaternionNormError<1e-10 && all(result.maxWheelSpeed<=rw.maxSpeed*(1+1e-9));
    if strcmp(name,"desaturation")
        objective=result.wheelMomentum(end)<.999*result.wheelMomentum(1);
    elseif strcmp(name,"fault")
        objective=any(strcmp(result.modes,"fault")) && any(strcmp(result.modes(ceil(.8*numel(result.modes)):end),"nominal"));
    else
        objective=rad2deg(result.pointingError(end))<3 && rad2deg(result.estimatorError(end))<3;
    end
    success=finite && bounded && objective;
end
