clear; clc;
testDir=fileparts(mfilename("fullpath"));
for folder={"config","dynamics","environment","math"}
    addpath(fullfile(testDir,"..",folder{1}));
end
sc=spacecraftParams(); rw=reactionWheelParams(); mtq=magnetorquerParams();
earth=earthEnvironmentParams(); model=truthModelParams();
names=fieldnames(model); for k=1:numel(names), model.(names{k})=false; end
r=earth.radius+400e3; v=sqrt(earth.mu/r);
x=[r;0;0;0;v;0;1;0;0;0;deg2rad([.2;-.1;.3]);[10;-20;30]];
motor=[1e-4;-2e-4;3e-4]; dipole=[.1;-.2;.3];
d=integratedSpacecraftDynamics(0,x,sc,rw,mtq,earth,model,motor,dipole);
limitedMotor=limitReactionWheelTorque(motor,x(14:16),rw);
fieldB=earthMagneticField(x(1:3),0,earth);
rodTorque=magnetorquerModel(dipole,fieldB,mtq);
H=sc.J*x(11:13)+rw.A*(rw.J.*x(14:16));
expectedOmega=sc.J\(rodTorque-rw.A*limitedMotor-cross(x(11:13),H));
assert(norm(d(1:6)-[x(4:6);twoBodyAcceleration(x(1:3),earth)])<1e-12);
assert(norm(d(11:13)-expectedOmega)<1e-12);
assert(norm(d(14:16)-limitedMotor./rw.J)<1e-12);
assert(all(isfinite(integratedSpacecraftDynamics(123,x,sc,rw,mtq,earth,truthModelParams(),zeros(3,1),zeros(3,1)))));
disp("ALL INTEGRATED SPACECRAFT DYNAMICS TESTS PASSED");
