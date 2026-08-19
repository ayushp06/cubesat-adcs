clear; clc; rng(7);
testDir = fileparts(mfilename("fullpath"));
addpath(fullfile(testDir,"..","config"));
addpath(fullfile(testDir,"..","sensors"));
addpath(fullfile(testDir,"..","math"));
s = sensorParams();

gyro = s.gyro; gyro.noiseStd=0; gyro.biasRandomWalkStd=0; gyro.scaleFactor=zeros(3,1); gyro.misalignment=zeros(3);
[y,b] = gyroMeasurement([1;2;3],[0.1;0.2;0.3],0.01,gyro);
assert(norm(y-[1.1;2.2;3.3]) < 1e-14 && norm(b-[0.1;0.2;0.3]) < 1e-14);
assert(all(abs(gyroMeasurement(10*ones(3,1),zeros(3,1),0.01,gyro)) <= gyro.maxRate));

mag=s.magnetometer; mag.noiseStd=0; mag.bias=zeros(3,1); mag.scaleFactor=zeros(3,1); mag.misalignment=zeros(3);
assert(norm(magnetometerMeasurement([20;-30;40]*1e-6,mag)-[20;-30;40]*1e-6) < 1e-15);

sun=s.sun; sun.noiseStd=0;
[direction,valid,currents]=coarseSunSensorMeasurement([1;2;3]/sqrt(14),1,sun);
assert(valid && norm(direction-[1;2;3]/sqrt(14)) < 1e-14 && nnz(currents)>0);
[direction,valid]=coarseSunSensorMeasurement([1;0;0],0,sun);
assert(~valid && all(isnan(direction)));

gps=s.gps; gps.positionNoiseStd=0; gps.velocityNoiseStd=0; gps.dropoutProbability=0;
[r,v,valid]=gpsMeasurement([1;2;3],[4;5;6],gps);
assert(valid && isequal(r,[1;2;3]) && isequal(v,[4;5;6]));

star=s.starTracker; star.enabled=true; star.angleNoiseStd=0; star.dropoutProbability=0;
[q,valid]=starTrackerMeasurement([1;0;0;0],star);
assert(valid && isequal(q,[1;0;0;0]));
disp("ALL SENSOR MODEL TESTS PASSED");
