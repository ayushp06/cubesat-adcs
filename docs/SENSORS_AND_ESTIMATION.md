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

## 5. Multiplicative extended Kalman filter

The MEKF is primary because adding four quaternion components in an ordinary
EKF conflicts with the unit-norm constraint. It keeps a normalized nominal
quaternion and estimates `delta_x = [delta_theta_B; delta_b_g]`. Locally,
`q_true = q_hat x [1; delta_theta/2]`.

The nominal attitude integrates the bias-corrected gyro:
`qdot_hat = 0.5 q_hat x [0; gyro_m - b_hat]`, with constant nominal bias.
The small-error model is

`delta_theta_dot = -[omega x] delta_theta - delta_b - n_g`,
`delta_b_dot = n_b`.

Therefore `F = [-[omega x], -I; 0, 0]` and the 100 Hz implementation uses
`Phi = I + F dt`. Configurable gyro white noise and bias random walk form the
process covariance. Use an exact matrix exponential if the sample period grows
large.

For inertial reference `r_I`, predicted body direction is
`b_hat = C_IB(q_hat)^T r_I`. The normalized-vector residual is
`y = b_measured - b_hat`, with `H = [[b_hat x], 0]`. Asynchronous vectors may
be stacked. The usual Kalman gain corrects the local attitude and bias; the
attitude correction is injected multiplicatively. The Joseph covariance form
preserves symmetry and positive semidefiniteness. Each update records the
innovation, innovation covariance, and normalized innovation squared (NIS).
No truth value enters these estimator functions.

An additive quaternion EKF is intentionally omitted: it is less natural for
the unit-quaternion manifold and would duplicate the MEKF without improving
the reference stack.

## 6. Noisy/dropout validation

`simulateMekfScenario(21)` keeps truth solely in its sensor-generation and
metric sections. The Sun sensor is unavailable from 20--35 s and the
magnetometer from 25--32 s, creating a seven-second gyro-only interval. Initial
attitude error is 20 degrees and gyro-bias error is about 0.39 deg/s.

GNU Octave on 2026-08-19 produced 0.8300 degree final attitude error, 1.3298
degree RMS error over the final ten seconds, and 0.01262 deg/s final bias
error. Covariance remained positive semidefinite (minimum eigenvalue
`1.330e-08`); maximum vector-update NIS was 53.768 during convergence. Run
`runMekfValidation` to reproduce all metrics.
