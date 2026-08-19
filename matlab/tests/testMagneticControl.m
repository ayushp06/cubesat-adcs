clear; clc;
testDir=fileparts(mfilename("fullpath"));
addpath(fullfile(testDir,"..","config"),fullfile(testDir,"..","control"), ...
    fullfile(testDir,"..","dynamics"),fullfile(testDir,"..","math"),fullfile(testDir,"..","simulations"));
mtq=magnetorquerParams(); rw=reactionWheelParams();
[torque,dipole]=magnetorquerModel(10*ones(3,1),[0;0;40e-6],mtq);
assert(all(abs(dipole)<=mtq.maxDipole) && abs(dot(torque,[0;0;40e-6]))<1e-20);
command=bDotController([1;2;3]*1e-5,zeros(3,1),1,mtq);
assert(all(command<0));

results=simulateBdotDetumble();
assert(results.finalRate<deg2rad(1) && results.finalRate<0.1*results.initialRate && results.saturated);

field=[20;-10;35]*1e-6; speed=0.8*rw.maxSpeed; initial=norm(rw.J.*speed);
for k=1:25000
    phase=2*pi*k/500; field=[30*cos(phase);20*sin(phase);25]*1e-6;
    [motor,dipole,magTorque]=momentumUnloadController(speed,field,rw,mtq);
    [checkTorque,~]=magnetorquerModel(dipole,field,mtq);
    assert(norm(checkTorque-magTorque)<1e-18 && all(abs(motor)<=rw.maxTorque+eps));
    speed=speed+0.2*(motor./rw.J);
end
assert(norm(rw.J.*speed)<0.2*initial && all(abs(speed)<rw.maxSpeed));
disp("ALL MAGNETIC CONTROL TESTS PASSED");
