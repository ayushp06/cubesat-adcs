function [flight,command] = flightAdcsStep(time,flight,packet,request,configuration)
% FLIGHTADCSSTEP Deterministic measurement-to-actuator flight boundary.
% packet contains sensor measurements/references and wheel telemetry only.
% No truth attitude or truth angular rate is accepted by this interface.
    flight.estimate=mekfPredict(flight.estimate,packet.gyro,packet.dt,configuration.mekf);
    if packet.magValid
        flight.estimate=mekfUpdateVectors(flight.estimate,packet.magBody,packet.magECI,configuration.mekf.magnetometerVectorNoiseStd);
    end
    if packet.sunValid
        flight.estimate=mekfUpdateVectors(flight.estimate,packet.sunBody,packet.sunECI,configuration.mekf.sunVectorNoiseStd);
    end
    omega=packet.gyro-flight.estimate.gyroBias;
    status=request.status; status.bodyRate=norm(omega);
    status.wheelSpeedFraction=max(abs(packet.wheelSpeed)./configuration.rw.maxSpeed);
    flight.mode=updateAdcsMode(flight.mode,status,configuration.mode);
    ownership=adcsModeCommand(flight.mode.mode);

    qReference=flight.lastReference; omegaReference=zeros(3,1);
    if strcmp(ownership.guidance,"safe")
        qReference=safeAttitude(packet.sunECI);
    elseif strcmp(ownership.guidance,"nominal")
        if strcmp(request.guidance,"nadir") && packet.gpsValid
            qReference=nadirPointing(packet.positionECI,packet.velocityECI);
            orbitRate=cross(packet.positionECI,packet.velocityECI)/dot(packet.positionECI,packet.positionECI);
            omegaReference=quatToDCM(qReference)'*orbitRate;
        else
            qReference=inertialPointing(request.qTarget);
        end
    elseif strcmp(ownership.guidance,"slew")
        [qReference,omegaReference]=quaternionSlewReference(request.qStart,request.qTarget,time-request.slewStart,request.slewDuration);
    end
    flight.lastReference=qReference;

    motor=zeros(3,1); dipole=zeros(3,1); desiredTorque=zeros(3,1);
    if ownership.reactionWheels
        desiredTorque=quaternionPDController(qReference,flight.estimate.qIB,omega-omegaReference,configuration.controller);
        motor=allocateReactionWheelTorque(desiredTorque,configuration.rw);
    end
    if strcmp(flight.mode.mode,"desaturation") && packet.magValid
        [unloadMotor,dipole]=momentumUnloadController(packet.wheelSpeed,packet.magBody,configuration.rw,configuration.mtq);
        motor=motor+unloadMotor;
    elseif strcmp(flight.mode.mode,"detumble") && packet.magValid && flight.previousMagValid
        dipole=bDotController(packet.magBody,flight.previousMag,packet.magPeriod,configuration.mtq);
    end
    if packet.magValid, flight.previousMag=packet.magBody; flight.previousMagValid=true; end
    command.motorTorque=limitReactionWheelTorque(motor,packet.wheelSpeed,configuration.rw);
    [~,command.dipole]=magnetorquerModel(dipole,packet.magBody,configuration.mtq);
    command.qReference=qReference; command.omegaReference=omegaReference;
    command.desiredBodyTorque=desiredTorque; command.mode=flight.mode.mode;
end
