function results = simulateControllerComparison()
% SIMULATECONTROLLERCOMPARISON Identical saturated 90-degree slew for PD/PID/LQR.
    sc=spacecraftParams(); rw=reactionWheelParams(); pd=controllerParams(sc); lqr=lqrControllerParams(sc);
    qReference=[cos(pi/4);sin(pi/4);0;0]; methods={"PD","PID","LQR"};
    times=linspace(0,180,1801); results=struct([]);
    for methodIndex=1:3
        x0=[1;0;0;0;zeros(3,1);zeros(3,1)];
        [t,x]=ode45(@dynamics,times,x0,odeset("RelTol",1e-8,"AbsTol",1e-10));
        error=zeros(numel(t),1); effort=zeros(numel(t),1);
        for k=1:numel(t)
            qError=quaternionAttitudeError(qReference,x(k,1:4)'); error(k)=2*acos(min(1,qError(1)));
            effort(k)=norm(command(x(k,:)',methods{methodIndex}));
        end
        settled=find(error<deg2rad(1) & vecnorm(x(:,5:7),2,2)<deg2rad(0.1),1);
        results(methodIndex).method=methods{methodIndex};
        results(methodIndex).finalErrorDeg=rad2deg(error(end));
        results(methodIndex).settlingTime=ternaryTime(settled,t);
        results(methodIndex).peakRateDegPerSec=rad2deg(max(vecnorm(x(:,5:7),2,2)));
        results(methodIndex).controlEffort=trapz(t,effort);
        results(methodIndex).peakTorque=max(effort);
    end

    function dx=dynamics(~,x)
        torque=command(x,methods{methodIndex});
        attitude=attitudeDynamics(0,x(1:7),sc,torque);
        qError=quaternionAttitudeError(qReference,x(1:4));
        integralDot=2*qError(2:4);
        if ~strcmp(methods{methodIndex},"PID") || any(abs(torque)>=rw.maxTorque), integralDot=zeros(3,1); end
        dx=[attitude;integralDot];
    end
    function torque=command(x,method)
        if strcmp(method,"PD")
            torque=quaternionPDController(qReference,x(1:4),x(5:7),pd);
        elseif strcmp(method,"PID")
            [torque,~]=quaternionPIDController(qReference,x(1:4),x(5:7),x(8:10),0,pd,rw.maxTorque);
        else
            torque=quaternionLqrController(qReference,x(1:4),x(5:7),lqr);
        end
        torque=max(-rw.maxTorque,min(rw.maxTorque,torque));
    end
end

function value=ternaryTime(index,times)
    if isempty(index), value=NaN; else, value=times(index); end
end
