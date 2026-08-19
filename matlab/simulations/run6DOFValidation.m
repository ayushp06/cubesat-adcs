clear; clc;
simulationDir = fileparts(mfilename("fullpath"));
addpath(fullfile(simulationDir,"..","config"));
addpath(fullfile(simulationDir,"..","dynamics"));
addpath(fullfile(simulationDir,"..","environment"));
addpath(fullfile(simulationDir,"..","math"));

earth = earthEnvironmentParams(); sc = spacecraftParams(); model = truthModelParams();
radius = earth.radius + 400e3; speed = sqrt(earth.mu/radius);
x0 = [radius;0;0; 0;speed*cos(deg2rad(51.6));speed*sin(deg2rad(51.6)); ...
      1;0;0;0; deg2rad([0.1;-0.05;0.2])];
period = 2*pi*sqrt(radius^3/earth.mu);
[time,state] = ode45(@(t,x) fullSpacecraftDynamics(t,x,sc,earth,model), ...
    linspace(0,period,1001),x0,odeset("RelTol",1e-9,"AbsTol",1e-8));

fprintf("Duration: %.3f s\n",time(end));
fprintf("Final altitude: %.3f km\n",(norm(state(end,1:3))-earth.radius)/1e3);
fprintf("Maximum quaternion norm error: %.3e\n", ...
    max(abs(vecnorm(state(:,7:10),2,2)-1)));
fprintf("Final body rate: [%.6e %.6e %.6e] rad/s\n",state(end,11:13));
