# Sensors and Attitude Estimation

## 1. Architecture and truth boundary

The 13-state plant is simulation truth. Only functions under `matlab/sensors`
may convert truth quantities into measurements. Estimators accept measured
angular rate/vector observations and model reference vectors; they never accept
the truth quaternion, truth body rate, or plant state. This mirrors flight
software, where truth does not exist.

All internal units are SI and all vectors follow `matlab/CONVENTIONS.md`.
Sample periods are configuration metadata used by scenario schedulers. A sensor
function produces exactly one sample when called, avoiding hidden clocks.

## 2. Sensor equations and assumptions

A three-axis sensor uses

`y = (I + diag(s)) (I + M) x + b + n`,

where `s` is scale-factor error, `M` is small cross-axis misalignment, `b` is
bias, and `n` is zero-mean Gaussian sample noise. Gyro bias evolves as
`b[k+1] = b[k] + sigma_rw sqrt(dt) w[k]`. Gyro and magnetometer outputs are
component-wise saturated.

Six coarse Sun sensors point along the positive and negative body axes. Each
has a cosine response `max(0, n_i dot s_B)`, additive current noise, and a
detection threshold. Eclipse produces an invalid vector. This captures the
hemispherical field of view and dropout without modeling electronics.

GPS returns noisy ECI position and velocity or an explicit invalid sample.
The optional star tracker returns a noisy attitude quaternion or an explicit
dropout; it is disabled by default.

## 3. Parameter traceability

No flight hardware has been selected. Values in `sensorParams.m` are therefore
project design assumptions, not fabricated datasheet specifications:

| Requirement | Assumption |
|---|---|
| SEN-GYR-001 | 100 Hz, 0.02 deg/s sample noise, 0.002 deg/s/sqrt(s) bias walk, 250 deg/s range |
| SEN-MAG-001 | 10 Hz, 150 nT sample noise, 100 microtesla range |
| SEN-SUN-001 | 5 Hz, six cosine-response faces, 0.05 normalized threshold |
| SEN-GPS-001 | 1 Hz, 3 m position and 0.05 m/s velocity sample noise, 1% dropout |
| SEN-STR-001 | Optional 1 Hz, 0.01 deg attitude noise, 2% dropout |

Replace these entries with traced datasheet or measured values when hardware
is selected; the sensor interfaces and tests remain unchanged.

## 4. Deterministic vector attitude solutions

Attitude is observable from two non-collinear reference directions. TRIAD
normalizes the first vector, forms the second axis from the cross product, and
completes right-handed orthonormal bases in B and I. The attitude matrix is
`C_IB = T_I T_B^T`. Collinear inputs are rejected because they contain no
rotation information about their common axis.

QUEST solves Wahba's problem: find the rotation minimizing
`0.5 sum(a_i |b_i - C_BI r_i|^2)`. `questAttitude` constructs Davenport's
symmetric 4-by-4 K matrix and selects the unit eigenvector of its largest
eigenvalue. The result is conjugated from reference-to-body into the repository
`q_IB` convention. This eigenvalue form is the q-method reference solution;
for the small vector counts here it is clearer than a specialized scalar-root
iteration and has the same Wahba optimum.
