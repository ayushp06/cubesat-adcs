function results=simulateMomentumUnloading()
% SIMULATEMOMENTUMUNLOADING Reproducible saturated three-wheel desaturation.
    rw=reactionWheelParams(); mtq=magnetorquerParams(); dt=0.2; steps=25000;
    speed=0.8*rw.maxSpeed; initial=norm(rw.J.*speed);
    sampleEvery=100; history=zeros(floor(steps/sampleEvery)+1,2); history(1,:)=[0,initial]; row=2;
    for k=1:steps
        phase=2*pi*k/500; field=[30*cos(phase);20*sin(phase);25]*1e-6;
        [motor,~,~]=momentumUnloadController(speed,field,rw,mtq);
        speed=speed+dt*(motor./rw.J);
        if mod(k,sampleEvery)==0
            history(row,:)=[k*dt,norm(rw.J.*speed)]; row=row+1;
        end
    end
    results.time=history(:,1); results.momentum=history(:,2);
    results.initialMomentum=initial; results.finalMomentum=norm(rw.J.*speed);
    results.finalFraction=results.finalMomentum/initial;
end
