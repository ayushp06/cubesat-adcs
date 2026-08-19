function results=runNadirPointingDemo()
% RUNNADIRPOINTINGDEMO Verify LVLH guidance around one inclined orbit.
    earth=earthEnvironmentParams(); radius=earth.radius+400e3; speed=sqrt(earth.mu/radius);
    inclination=deg2rad(51.6); x0=[radius;0;0;0;speed*cos(inclination);speed*sin(inclination)];
    period=2*pi*sqrt(radius^3/earth.mu);
    [time,state]=ode45(@(t,x) orbitalDynamics(t,x,earth,true),linspace(0,period,361),x0, ...
        odeset("RelTol",1e-9,"AbsTol",1e-7));
    nadirError=zeros(numel(time),1); orthogonality=zeros(numel(time),1);
    for k=1:numel(time)
        C=quatToDCM(nadirPointing(state(k,1:3)',state(k,4:6)'));
        nadir=-state(k,1:3)'/norm(state(k,1:3));
        nadirError(k)=acos(max(-1,min(1,dot(C(:,3),nadir))));
        orthogonality(k)=norm(C'*C-eye(3),"fro");
    end
    results.time=time; results.maxNadirError=max(nadirError);
    results.maxOrthogonalityError=max(orthogonality);
end
