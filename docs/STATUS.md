# CubeSat ADCS Project Status

Updated: 2026-08-19

## Current phase

Phase 1 — Dynamics foundation and automated validation.

Stage 0 project memory is established. The one-wheel reaction-wheel parameter
interface is repaired and covered by an analytical regression test. The next
milestone is broader automated dynamics invariant testing.

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
- Research-grade theory and implementation guide in `README.md`.

## Known incomplete or unvalidated work

- An uncommitted pre-existing edit in `attitudeDynamics3RW.m` adds wheel-speed
  limiting but duplicates the wheel-acceleration calculation; it is not yet
  validated or included in Stage 0 commits.
- Dynamics invariants are plotted or printed, not enforced by automated tests.
- Three-wheel speed-limit behavior has no regression test.
- Default `ode45` tolerances are used without a convergence study.
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
| `runReactionWheelTest` | PASS — max momentum error `4.337e-19 N m s` |

These results describe only the commands above; no unrun result is implied.

## Next tasks

1. Add one automated dynamics test covering analytical principal-axis motion,
   internal momentum conservation, torque clipping, and speed limiting.
2. Validate and clean the existing three-wheel speed-limit edit.
3. Record explicit ODE tolerances and convergence evidence.

## Latest good commit

`5cbb579` (`fix: restore one-wheel dynamics simulation`) is the latest validated
technical commit. The working tree also contains the pre-existing uncommitted
three-wheel speed-limit edit described above.
