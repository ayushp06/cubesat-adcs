function results = simulateBdotDetumble()
% SIMULATEBDOTDETUMBLE Fixed-step magnetic detumble with a changing LEO field.
    sc=spacecraftParams(); mtq=magnetorquerParams(); dt=0.2; times=0:dt:10000;
    q=[1;0;0;0]; omega=deg2rad([8;-6;5]); rates=zeros(numel(times),1);
    previousField=quatToDCM(q)'*[30;5;25]*1e-6;
    saturated=false;
    for k=1:numel(times)
        phase=2*pi*times(k)/5400;
        fieldECI=1e-6*[30*cos(phase);20*sin(phase);25];
        fieldB=quatToDCM(q)'*fieldECI;
        dipole=bDotController(fieldB,previousField,dt,mtq);
        [torque,applied]=magnetorquerModel(dipole,fieldB,mtq);
        saturated=saturated || any(abs(applied)>=mtq.maxDipole-1e-12);
        omega=omega+dt*(sc.J\(torque-cross(omega,sc.J*omega)));
        angle=norm(omega)*dt;
        if angle>0, q=quatNormalize(quatMultiply(q,[cos(angle/2);sin(angle/2)*omega/norm(omega)])); end
        rates(k)=norm(omega); previousField=fieldB;
    end
    results.times=times; results.rates=rates; results.finalRate=norm(omega);
    results.initialRate=norm(deg2rad([8;-6;5])); results.saturated=saturated;
end
