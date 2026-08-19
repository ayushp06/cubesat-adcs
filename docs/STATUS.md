# CubeSat ADCS Project Status

Updated: 2026-08-19

## Current phase

The portable C++ algorithm candidate is implemented and passing incremental
GoogleTest/MATLAB-vector checks. FSW-003 remains PARTIAL until the required
clean build and complete relevant-suite evidence are recorded.

Stage 0 project memory is established. The one-wheel reaction-wheel parameter
interface is repaired and covered by an analytical regression test. Automated
plant invariants and shared reaction-wheel actuator limits are now validated.
Quaternion error, PD feedback, three-wheel allocation, closed-loop slew
scenarios, performance metrics, and representative solver convergence are
validated. The MATLAB attitude-control foundation requested for Stage 1 is
complete. ECI two-body/J2 translation, modular force/torque environments, and
coupled 13-state 6-DOF propagation are validated. Flight-like gyro,
magnetometer, coarse-Sun, GPS, and optional star-tracker models are validated.
TRIAD, QUEST, and the primary gyro-bias MEKF are validated without exposing
truth state to estimator functions.

## Completed and validated

- Repository-wide scalar-first Hamilton quaternion convention: `q_IB` rotates
  body-frame coordinates into the inertial frame.
- SI-unit spacecraft and three-wheel parameter configuration.
- Quaternion multiplication, conjugation, normalization, and body-to-inertial
  DCM conversion.
- Seven quaternion regression assertions covering identities, normalization,
  inverse behavior, DCM orthogonality/determinant, and a known +90-degree
  rotation.
- Nonlinear torque-free rigid-body dynamics and quaternion kinematics.
- Ideal three-wheel rigid-body coupling, motor-torque clipping, and an
  executable demonstration.
- One-wheel momentum exchange using the first configured wheel, checked against
  its analytical acceleration and total-momentum invariant.
- Shared reaction-wheel torque clipping and directional speed limiting; braking
  commands remain available at either speed boundary.
- Automated general torque-free energy/inertial-momentum and three-wheel total
  momentum regression checks with explicit ODE tolerances.
- Shortest-path quaternion attitude error, inertia-scaled quaternion PD control,
  and sign-correct three-wheel torque allocation.
- Closed-loop 90-degree X/Y/Z slews, an arbitrary-axis slew, and an
  arbitrary-axis slew with nonzero initial rates on all body axes.
- Metrics for pointing error, settling time, overshoot, body rate, applied
  control effort, wheel speed, and wheel momentum.
- Explicit ODE tolerances in simulations and a two-tolerance convergence check.
- ECI position/velocity propagation with two-body gravity and optional J2,
  including analytical J2 checks and orbital energy/momentum regression tests.
- Independent gravity-gradient, exponential-atmosphere drag, aerodynamic
  torque, analytic Sun, cylindrical eclipse, solar-pressure, centered-dipole
  magnetic-field, and residual-dipole torque models.
- Coupled 13-state ECI translation and body rotation with independently
  selectable environment effects and no estimator/flight-state dependency.
- Sampled gyro, magnetometer, six-face coarse-Sun, GPS, and optional
  star-tracker models with explicit calibration errors, noise, limits, and
  dropout behavior.
- TRIAD and Davenport q-method/QUEST reference-vector attitude solutions with
  analytical known-attitude and invalid-geometry checks.
- Six-state right-multiplicative MEKF with asynchronous vector updates, gyro
  bias estimation, Joseph covariance update, and innovation/NIS tracking.
- Inertial, Sun/safe, nadir/LVLH, and cubic-time shortest-path slew guidance.
- Bounded quaternion PID-style integral augmentation and CARE-derived LQR with
  controllability, stability, and PD/PID/LQR performance checks.
- Three-axis saturated magnetorquer actuation, B-dot detumbling, and
  field-achievable reaction-wheel momentum unloading.
- Priority-ordered initialization/detumble/safe/nominal/slew/desaturation/fault
  mode management with hysteresis and explicit actuator ownership.
- Minimum-norm reaction-wheel allocation validated for orthogonal, redundant,
  and one-wheel-degraded full-rank geometries.
- Versioned CRC-protected SI-unit sensor/actuator protocol, rollover-safe timing
  contract, portable C++ quaternion PD reference, CMake, and native SIL test.
- Requirements-to-test matrix, seeded Monte Carlo and fault scenarios, nadir
  and desaturation demonstrations, consolidated CSV results, metadata, and
  generated plots.
- Portable C++ quaternion/frame math, QUEST/gyro-bias MEKF, guidance, PD/LQR,
  allocation, mode logic, explicit configuration, and estimate-to-actuator API.
- Research-grade theory and implementation guide in `README.md`.

## Known incomplete or unvalidated work

- The new C++ port has not yet passed its final clean verification gate.
  Simulink and physical HIL are not implemented or executed. Environment,
  sensor, and actuator models use engineering assumptions rather than selected
  flight-hardware data.
- No single scenario closes the full plant-to-sensor-to-estimator-to-guidance/
  mode-to-actuator loop. Existing demonstrations validate those components in
  isolation or in smaller closed loops.

## Validation status

Executed with GNU Octave on 2026-08-19:

| Check | Result |
|---|---|
| `testQuaternionMath` | PASS — 7 assertions |
| `runTorqueFreeRotation` | PASS — max normalized quaternion error `2.220e-16` |
| `runThreeWheelTest` | PASS — completed without runtime error |
| `testReactionWheelDynamics` | PASS — derivative, final speed, and momentum assertions |
| `testDynamicsFoundation` | PASS — energy, inertial/total momentum, torque, and speed-limit assertions |
| `testAttitudeControl` | PASS — error convention, feedback signs, and allocation assertions |
| `testAttitudeSlews` | PASS — five scenarios, actuator bounds, metrics, and solver convergence |
| `runReactionWheelTest` | PASS — max momentum error `2.168e-19 N m s` |
| `runAttitudeSlewSuite` | PASS — five scenarios completed and metrics reported |
| `testOrbitDynamics` | PASS — one-orbit state/invariants and J2 known answers |
| `runOrbitValidation` | PASS — energy `7.236e-11`, momentum `3.618e-11` relative error |
| `testSpaceEnvironment` | PASS — force, torque, illumination, and field checks |
| `testFullSpacecraftDynamics` | PASS — coupled derivative, orbit invariants, quaternion norm |
| `run6DOFValidation` | PASS — 5553.624 s all-effects propagation, quaternion error `1.336e-08` |
| `testSensorModels` | PASS — deterministic sensor equations, limits, and dropout |
| `testAttitudeDetermination` | PASS — TRIAD, QUEST, DCM conversion, collinearity rejection |
| `testMekf` | PASS — propagation, vector correction, noisy bias/dropout scenario, covariance/NIS |
| `runMekfValidation` | PASS — final attitude `0.8300 deg`, bias error `0.01262 deg/s` |
| `testGuidance` | PASS — pointing geometry, LVLH frame, slew endpoints/rates |
| `testAdvancedControllers` | PASS — PID limits, LQR controllability/CARE/stability, comparison |
| `runControllerComparison` | PASS — PD/PID/LQR saturated 90-degree slew metrics |
| `testMagneticControl` | PASS — rod/wheel limits, B-dot detumble, momentum reduction |
| `testModeManagement` | PASS — all transitions, hysteresis, actuator ownership, degraded allocation |
| CMake/`adcs_sil` | PASS — release build, packet sizes/CRC, timer rollover, PD signs |
| `testVerificationScenarios` | PASS — nadir geometry, fault sequence, two seeded MEKF runs |
| `runProjectVerification` | PASS — 15 checks, 12/12 Monte Carlo, SIL, plots/tables |
| Incremental C++ GoogleTest | PASS — 9 tests including MATLAB reference-vector parity |

These results describe only the commands above; no unrun result is implied.

## Next tasks

1. Port the validated MEKF, guidance, and mode-management algorithms to C++.
2. Execute processor/HIL tests only when physical hardware is available.

## Latest good commit

`d9acbf5` (`docs: publish final verified project evidence`) is the audited
implementation baseline. The completion-audit commit changes evidence and
status documentation only.
