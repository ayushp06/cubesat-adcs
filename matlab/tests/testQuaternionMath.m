clear;
clc;

addpath(fullfile(fileparts(mfilename("fullpath")),"..","math"));

tol = 1e-12;

%% TEST 1: Identity quaternion

qIdentity = [1; 0; 0; 0];

q = quatMultiply(qIdentity, qIdentity);

assert(norm(q - qIdentity) < tol);

disp("Test 1 passed: identity multiplication");


%% TEST 2: Quaternion normalization

qRaw = [2; 0; 0; 0];

q = quatNormalize(qRaw);

assert(abs(norm(q) - 1) < tol);
assert(norm(q - qIdentity) < tol);

disp("Test 2 passed: quaternion normalization");


%% TEST 3: Quaternion conjugate

q = quatNormalize([1; 2; 3; 4]);

qConj = quatConjugate(q);

result = quatMultiply(q, qConj);

assert(norm(result - qIdentity) < tol);

disp("Test 3 passed: quaternion inverse property");


%% TEST 4: Identity DCM

C = quatToDCM(qIdentity);

assert(norm(C - eye(3), "fro") < tol);

disp("Test 4 passed: identity DCM");


%% TEST 5: DCM orthogonality

q = quatNormalize([0.8; 0.2; -0.3; 0.4]);

C = quatToDCM(q);

assert(norm(C*C' - eye(3), "fro") < tol);

disp("Test 5 passed: DCM orthogonality");


%% TEST 6: DCM determinant

assert(abs(det(C) - 1) < tol);

disp("Test 6 passed: DCM determinant");

%% TEST 7: 90-degree rotation about +Z

theta = deg2rad(90);

qZ90 = [
    cos(theta/2)
    0
    0
    sin(theta/2)
    ];

C = quatToDCM(qZ90);

vB = [1; 0; 0];

vI = C * vB;

expected = [0; 1; 0];

assert(norm(vI - expected) < tol);

disp("Test 7 passed: +90 deg Z rotation");

disp(" ");
disp("ALL QUATERNION TESTS PASSED");
