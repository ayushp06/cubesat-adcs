# CubeSat ADCS Project Status

Updated: 2026-08-19

## Current phase

Phase 2 — orbit and disturbance truth models — is in progress.

Stage 0 project memory is established. The one-wheel reaction-wheel parameter
interface is repaired and covered by an analytical regression test. Automated
plant invariants and shared reaction-wheel actuator limits are now validated.
Quaternion error, PD feedback, three-wheel allocation, closed-loop slew
scenarios, performance metrics, and representative solver convergence are
validated. The MATLAB attitude-control foundation requested for Stage 1 is
complete. ECI two-body/J2 translation and modular force/torque environments are
now validated; coupled 6-DOF propagation is next.

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
- Research-grade theory and implementation guide in `README.md`.

## Known incomplete or unvalidated work

- Orbit, environmental torques, sensors, estimation, pointing-mode guidance,
  detumble, momentum dumping, C++ flight software, CMake, and Simulink
  integration are not implemented. The current controller uses truth state and
  a fixed target.

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

These results describe only the commands above; no unrun result is implied.

## Next tasks

1. Couple translation and rotation in the 13-state 6-DOF truth propagator.
3. Preserve the current truth-state controller as the control-law reference
   when sensor and estimator states are introduced.

## Latest good commit

`eda7348` (`feat: validate closed-loop attitude slews`) is the latest validated
technical commit.
