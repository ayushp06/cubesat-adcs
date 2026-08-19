# CubeSat ADCS Roadmap

This roadmap advances the repository from its validated MATLAB dynamics
foundation toward a research-grade digital twin and portable flight software.
Each phase requires documented conventions, analytical or numerical checks,
automated regression tests, and an updated project status before completion.

## Phase 0 — Persistent project memory

- [x] Record permanent engineering and Git rules in `AGENTS.md`.
- [x] Inventory the repository, history, and current validation state.
- [x] Establish this roadmap and `docs/STATUS.md`.

## Phase 1 — Dynamics foundation and automated validation

- [x] Define scalar-first Hamilton `q_IB` and body-rate conventions.
- [x] Implement and test quaternion algebra and DCM conversion.
- [x] Implement torque-free rigid-body propagation.
- [x] Implement ideal one-wheel and three-wheel momentum exchange.
- [x] Repair the one-wheel parameter interface regression.
- [x] Add automated rigid-body and reaction-wheel invariant tests.
- [x] Complete and test reaction-wheel torque and speed saturation behavior.
- [x] Set explicit solver tolerances and document convergence evidence.

## Phase 2 — Orbit and disturbance truth models

- [x] Add a validated ECI two-body/J2 orbit propagator and time handling.
- [ ] Add gravity-gradient torque with analytical limiting cases.
- [ ] Add configurable aerodynamic, solar-pressure, and magnetic disturbances.
- [ ] Keep environment and spacecraft truth state independent of flight state.

## Phase 3 — Sensor truth and measurement models

- [ ] Add gyroscope, magnetometer, and Sun-sensor truth interfaces.
- [ ] Model bias, noise, scale factor, misalignment, sampling, and saturation.
- [ ] Trace parameters to requirements, datasheets, or measured hardware.
- [ ] Validate deterministic limits before stochastic Monte Carlo testing.

## Phase 4 — Attitude determination

- [ ] Implement reference-vector attitude determination.
- [ ] Implement a quaternion estimator with explicit covariance conventions.
- [ ] Validate perfect-measurement, biased, noisy, and dropout cases.
- [ ] Report estimation error without conflating truth and estimated state.

## Phase 5 — Guidance and control

- [ ] Define pointing modes and reference-attitude generation.
- [x] Implement and analytically validate quaternion attitude error.
- [ ] Add a dedicated detumble controller.
- [x] Add three-axis quaternion PD slew control.
- [x] Verify signs, frames, settling time, and actuator usage.

## Phase 6 — Actuator allocation and momentum management

- [x] Allocate body torque to the current orthogonal three-wheel assembly.
- [ ] Generalize wheel allocation for the selected physical wheel geometry.
- [ ] Add magnetorquer or other momentum-dumping actuation.
- [ ] Implement wheel desaturation and degraded-actuator cases.
- [ ] Validate momentum, power, and authority limits.

## Phase 7 — Flight-software implementation

- [ ] Define deterministic, unit-explicit C++ interfaces.
- [ ] Port validated estimation and control algorithms from the truth model.
- [ ] Add CMake builds and automated C++ unit/integration tests.
- [ ] Cross-check MATLAB/Simulink and C++ outputs from shared test vectors.

## Phase 8 — Integrated digital twin

- [ ] Connect orbit, environment, sensors, estimator, guidance, control, and actuators.
- [ ] Add Simulink integration where block-level simulation adds value.
- [ ] Run scenario, fault, sensitivity, and Monte Carlo campaigns.
- [ ] Record solver, seed, configuration, commit, and reproducibility metadata.

## Phase 9 — Hardware calibration and qualification evidence

- [ ] Replace nominal parameters with measured mass and actuator properties.
- [ ] Add hardware-, processor-, or software-in-the-loop testing as available.
- [ ] Quantify uncertainty and compare predictions with experimental data.
- [ ] Document the model's validated operating envelope and remaining limits.
