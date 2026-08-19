# Results in Plain English

## Bottom line

The simulated CubeSat completed all five end-to-end test missions. In each
test, the software used noisy virtual sensors to estimate its orientation,
decide where to point, command the reaction wheels or magnetorquers, and move
the simulated spacecraft. The estimator and controller were not given the
true attitude or true rotation rate as a shortcut.

These results show that the software components work together in the current
simulation. They do **not** prove that a real spacecraft will perform the same
way. The hardware values and environment models are engineering assumptions,
and no physical hardware was tested.

## What was tested

| Test | Everyday meaning | Result |
|---|---|---|
| Commanded slew | Turn to a new orientation smoothly | PASS |
| Inertial pointing | Hold a fixed direction in space | PASS |
| Nadir pointing | Keep the spacecraft pointed toward Earth while orbiting | PASS |
| Desaturation | Use magnetorquers to begin removing stored wheel momentum | PASS |
| Fault recovery | Handle a sensor outage, enter fault mode, reset, and resume pointing | PASS |

Each run used a fixed random seed, so another user can reproduce the same
noise and results.

## Key numbers and what they mean

### Pointing accuracy

Final pointing error ranged from **0.484° to 0.894°**. An error of about one
degree is comparable to missing a target by roughly 1.7 cm at a distance of
one metre. Every controlled scenario finished below the test limit of 3°.

| Scenario | Final pointing error | Plain-English interpretation |
|---|---:|---|
| Slew | 0.584° | Reached the commanded orientation within one degree |
| Inertial | 0.894° | Held the requested direction within one degree |
| Nadir | 0.729° | Tracked the moving Earth-pointing direction within one degree |
| Desaturation | 0.693° | Maintained useful pointing while unloading the wheels |
| Fault | 0.484° | Recovered from the injected fault and returned to accurate pointing |

### Time to settle

“Settling time” is when pointing error stays below 2° and body-rate error stays
below 0.2°/s for the rest of the run.

| Scenario | Settling time |
|---|---:|
| Slew | 65.36 s |
| Inertial | 41.50 s |
| Nadir | 53.72 s |
| Fault recovery | 46.64 s |

The desaturation run reports 0 s because it began inside the attitude-settling
limits. That value does not mean wheel unloading finished instantly; wheel
momentum is assessed separately.

### Attitude-estimator accuracy

The MEKF finished with **0.570° to 0.971°** attitude error across the five
runs. This means the flight-side software reconstructed the spacecraft
orientation to about one degree despite sensor noise and scheduled sensor
updates. During the fault scenario, vector sensors and GPS were deliberately
unavailable for part of the run, yet the mode logic recovered and the final
estimator error was 0.570°.

### Reaction wheels and saturation

The slew test reached the configured motor-torque limit. This is intentional:
it proves the complete loop remains stable when the requested torque is larger
than the actuator can provide. Wheel speeds remained inside the configured
speed limit in every run.

The desaturation scenario began with high wheel momentum and reduced it from
**0.0412650 to 0.0410452 N·m·s** during 180 seconds. This is a small reduction,
about **0.53%**, but it demonstrates the correct direction of momentum transfer
in the integrated loop. It is not evidence of complete unloading in 180
seconds.

### Numerical health

The largest quaternion norm error was **3.331 × 10⁻¹⁶**, essentially floating-
point round-off. In simple terms, the mathematical representation of attitude
remained valid throughout every simulation.

The native C++ flight stack also passed a clean strict build, all **11
GoogleTests**, and its MATLAB comparison tests. All **18 MATLAB/Octave test
scripts** passed.

## What “PASS” does and does not mean

A PASS means the scenario executed the complete software loop and met its
documented numerical limits. It supports the claim that the algorithms are
integrated correctly for these models, configurations, seeds, and durations.

A PASS does not establish:

- flight qualification or mission reliability;
- performance on a real microcontroller;
- real sensor, reaction-wheel, or magnetorquer accuracy;
- calibrated spacecraft mass and inertia;
- real serial communication or physical hardware-in-the-loop operation; or
- performance under every orbit, disturbance, failure, or manufacturing error.

Those claims require selected hardware, calibration data, processor timing,
larger uncertainty campaigns, and physical testing.

## Where the evidence lives

- Numerical table: `artifacts/integration/integrated_results.csv`
- Pointing plot: `artifacts/integration/integrated_pointing.png`
- Estimator plot: `artifacts/integration/integrated_estimator.png`
- Desaturation plot: `artifacts/integration/integrated_desaturation.png`
- Campaign log: `artifacts/integration/integration_campaign.log`
- Full audit: `docs/PROJECT_COMPLETION_CHECKLIST.md`

To reproduce the results from the repository root:

```bash
octave --quiet --eval 'addpath(genpath("matlab")); runIntegratedAdcsCampaign("artifacts/integration");'
```
