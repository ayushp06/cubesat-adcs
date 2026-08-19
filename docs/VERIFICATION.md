# Requirements, Verification, and Reproducibility

## Verification policy

PASS means the named automated check or seeded scenario was actually executed
on the recorded software revision. Analytical checks compare with a closed-form
result; numerical checks enforce a stated threshold. Model-to-model agreement
is not physical validation. No physical HIL, calibrated flight hardware, or
on-orbit data were used.

## Requirement-to-test traceability

| ID | Requirement | Method and acceptance | Automated evidence | Status |
|---|---|---|---|---|
| MAT-001 | Scalar-first Hamilton `q_IB`, B to ECI | identities, known rotation, proper DCM | `testQuaternionMath` | PASS |
| DYN-001 | Torque-free attitude | energy/momentum within tolerances | `testDynamicsFoundation`, final campaign | PASS |
| DYN-002 | Wheel/body momentum exchange | error `<1e-12 N m s` | `testReactionWheelDynamics`, campaign | PASS |
| ORB-001 | ECI two-body/J2 orbit | invariants and analytical J2 cases | `testOrbitDynamics` | PASS |
| ENV-001 | Modular LEO environment | analytical force/torque/eclipse/field cases | `testSpaceEnvironment` | PASS |
| DYN-003 | Coupled 13-state 6-DOF | composition and quaternion error `<1e-6` | `testFullSpacecraftDynamics`, campaign | PASS |
| SEN-001 | Gyro/mag/Sun/GPS/star tracker | limits, noise/calibration/dropout interfaces | `testSensorModels` | PASS |
| EST-001 | Two-vector attitude | known attitude; reject collinearity | `testAttitudeDetermination` | PASS |
| EST-002 | Attitude/bias estimate without truth input | RMS `<2 deg`, bias `<0.08 deg/s` | `testMekf`, campaign | PASS |
| GNC-001 | Inertial/Sun/safe/LVLH/slew guidance | geometry and boundary conditions | `testGuidance`, `testVerificationScenarios` | PASS |
| GNC-002 | Bounded three-axis wheel control | five slews, error `<0.01 deg`, limits | `testAttitudeSlews` | PASS |
| GNC-003 | PD/PID/LQR common comparison | rank 6, stable poles, settle `<180 s` | `testAdvancedControllers`, campaign | PASS |
| MAG-001 | Saturated 3-axis B-dot | rate `<1 deg/s` and `<10%` initial | `testMagneticControl`, campaign | PASS |
| MOM-001 | Unload wheel momentum | fraction `<0.2`, limits obeyed | `testMagneticControl`, campaign | PASS |
| MOD-001 | Seven deterministic modes | transitions, hysteresis, fault latch | `testModeManagement`, fault demo | PASS |
| FSW-001 | SI-unit transport/timing | packet sizes, CRC, timer rollover; no live transport | `adcs_sil` | PARTIAL |
| FSW-002 | Portable control convention | known PD direction/damping | MATLAB test and `adcs_sil` | PASS (host SIL) |
| FSW-003 | Portable flight algorithm stack | strict build; GoogleTest; MATLAB parity within documented tolerances | `adcs_sil`, reference CSV, FSW-003 logs | PASS |
| INT-001 | Closed-loop digital twin | five seeded scenarios; pointing/MEKF `<3 deg`, quaternion error `<1e-10`, wheel limits; saturation, momentum reduction, fault/recovery | `testIntegratedAdcs`, integrated CSV/figures/log | PASS |
| SIM-001 | Simulink integration | add only where block-level simulation adds value | no demonstrated need | NOT APPLICABLE |
| VER-001 | Seeded noisy regression | 12 seeds meet EST-002 | `runProjectVerification` | PASS |
| HIL-001 | Physical MCU closed loop | timing/electrical/closed-loop evidence | none | BLOCKED |
| CAL-001 | Measured hardware parameters | selected hardware/calibration trace | none | BLOCKED |

## Final campaign

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
octave --quiet --eval 'addpath(genpath("matlab")); runProjectVerification("artifacts/verification");'
octave --quiet --eval 'addpath(genpath("matlab")); runIntegratedAdcsCampaign("artifacts/integration");'
```

MEKF headline seed 21 and Monte Carlo seeds 1001--1012 are fixed. Outputs are
`verification_results.csv`, `monte_carlo.csv`, and PNG figures under
`artifacts/verification`.

## Demonstration coverage

| Demonstration | Entry point/evidence |
|---|---|
| Torque-free validation | `runTorqueFreeRotation`, campaign CSV |
| Reaction-wheel exchange | `runReactionWheelTest`, campaign CSV |
| Closed-loop/arbitrary 3-axis slew | `runAttitudeSlewSuite`, `slew_error.png` |
| Full 6-DOF | `run6DOFValidation`, campaign CSV |
| Noisy sensors / MEKF | `runMekfValidation`, `mekf_error.png` |
| B-dot detumble | `simulateBdotDetumble`, `bdot_detumble.png` |
| Nadir pointing | `runNadirPointingDemo`, campaign CSV |
| Wheel desaturation | `simulateMomentumUnloading`, plot |
| PD/PID/LQR | `runControllerComparison`, campaign CSV |
| Monte Carlo | `runMonteCarloVerification`, CSV/histogram |
| Fault scenario | `runFaultScenario` |
| SIL | CMake/CTest `adcs_sil` |
| Full estimator-in-the-loop cases | `runIntegratedAdcsCampaign`, integrated CSV/figures |

## Verified results on 2026-08-19

| Result | Measured |
|---|---:|
| Torque-free relative energy error | `3.295e-10` |
| Wheel momentum error | `2.168e-19 N m s` |
| Arbitrary slew final error | `4.769e-05 deg` |
| 6-DOF quaternion norm error | `1.455e-08` |
| MEKF RMS / bias error | `1.3298 deg` / `0.01262 deg/s` |
| B-dot final rate | `0.08935 deg/s` |
| Nadir-axis maximum error | `3.650e-08 rad` |
| Final wheel-momentum fraction | `0.14725` |
| PD / PID / LQR settling | `52.6 / 52.6 / 63.4 s` |
| Monte Carlo pass rate | `12/12` |
| Native SIL | PASS |
| Integrated slew settling / final pointing | `65.36 s` / `0.5839 deg` |
| Integrated inertial / nadir final pointing | `0.8939 / 0.7293 deg` |
| Integrated MEKF final-error range | `0.5699--0.9711 deg` |
| Integrated desaturation momentum | `0.0412650 -> 0.0410452 N m s` |
| Integrated quaternion norm error maximum | `3.331e-16` |

These are numerical results for configured models, not flight guarantees.
