# CubeSat ADCS Project Status

Updated: 2026-08-19

## Current phase

Phase 1 — Dynamics foundation and automated validation.

Stage 0 project memory is established. The first incomplete technical
milestone is repairing the one-wheel reaction-wheel parameter interface,
followed by automated dynamics invariant tests.

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
- Research-grade theory and implementation guide in `README.md`.

## Known incomplete or unvalidated work

- The one-wheel simulation fails because its scalar dynamics interface receives
  three-element wheel parameter vectors.
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
| `runReactionWheelTest` | FAIL — scalar/vector interface mismatch at `attitudeDynamicsRW.m:55` |

These results describe only the commands above; no unrun result is implied.

## Next tasks

1. Repair the one-wheel interface using the first configured wheel explicitly.
2. Add one automated dynamics test covering analytical principal-axis motion,
   internal momentum conservation, torque clipping, and speed limiting.
3. Validate and clean the existing three-wheel speed-limit edit.
4. Record explicit ODE tolerances and convergence evidence.

## Latest good commit

`d485e9b` (`documentation`) is the latest committed baseline inspected before
Stage 0. The working tree also contains the pre-existing uncommitted
three-wheel speed-limit edit described above.
