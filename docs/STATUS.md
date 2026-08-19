# CubeSat ADCS Project Status

Updated: 2026-08-19

## Current phase

Phase 1 — Dynamics foundation and automated validation.

Stage 0 project memory is established. The one-wheel reaction-wheel parameter
interface is repaired and covered by an analytical regression test. Automated
plant invariants and shared reaction-wheel actuator limits are now validated.
The next milestone is quaternion feedback control.

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
- Research-grade theory and implementation guide in `README.md`.

## Known incomplete or unvalidated work

- Demonstration scripts still use default `ode45` tolerances; automated
  dynamics tests use explicit tolerances, but no convergence study exists yet.
- Orbit, environmental torques, sensors, estimation, guidance, control,
  momentum dumping, C++ flight software, CMake, and Simulink integration are
  not implemented.

## Baseline test status

Executed with GNU Octave on 2026-08-19:

| Check | Result |
|---|---|
| `testQuaternionMath` | PASS — 7 assertions |
| `runTorqueFreeRotation` | PASS — max normalized quaternion error `2.220e-16` |
| `runThreeWheelTest` | PASS — completed without runtime error |
| `testReactionWheelDynamics` | PASS — derivative, final speed, and momentum assertions |
| `testDynamicsFoundation` | PASS — energy, inertial/total momentum, torque, and speed-limit assertions |
| `runReactionWheelTest` | PASS — max momentum error `4.337e-19 N m s` |

These results describe only the commands above; no unrun result is implied.

## Next tasks

1. Implement and validate quaternion attitude error, PD control, and
   reaction-wheel allocation.
2. Add closed-loop slew scenarios and performance metrics.
3. Record a solver convergence study for representative closed-loop cases.

## Latest good commit

`5cbb579` (`fix: restore one-wheel dynamics simulation`) is the latest validated
technical commit. The working tree also contains the pre-existing uncommitted
three-wheel speed-limit edit described above.
