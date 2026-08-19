# Integrated Digital Twin

## Execution path and boundary

`simulateIntegratedAdcsScenario` closes the loop at 50 Hz with a fixed-step
RK4 truth propagation:

```text
16-state truth plant -> sensor models -> measurement packet -> flightAdcsStep
       ^                                            |
       |       MEKF -> mode/guidance -> PD -> allocation
       |                                            |
       +--- wheel motor torque and rod dipole -------+
```

The truth state is ECI position and velocity, scalar-first Hamilton `q_IB`,
body rate `omega_BI_B`, and three wheel speeds. `flightAdcsStep` accepts only
the sensor packet, wheel telemetry, mission request, and flight configuration.
Its signature has no truth quaternion, truth body rate, or plant state. Truth
is referenced outside that boundary only to generate measurements and compute
post-run verification metrics.

## Reused components

The plant composes the existing two-body/J2, drag, solar-pressure,
gravity-gradient, and magnetic environment modules. It adds the existing
reaction-wheel torque/speed limits and magnetorquer model to the rotational
equation

\[
J\dot\omega=\tau_{env}+m\times B-A\tau_w-
\omega\times(J\omega+A(J_w\Omega_w)),\qquad
\dot\Omega_w=J_w^{-1}\tau_w.
\]

The flight boundary calls the existing MEKF, guidance, mode manager,
quaternion PD, minimum-norm wheel allocation, B-dot, and momentum unloading
functions. Internally all frames and units follow `matlab/CONVENTIONS.md` and
SI. No alternate algorithm implementations were introduced.

## Timing and scenarios

Gyro/control/plant steps are 0.02 s; magnetometer, coarse-Sun, and GPS updates
retain their configured 0.1 s, 0.2 s, and 1 s rates. Commands are held through
each RK4 step. Seeds 3101--3105 reproduce slew, arbitrary inertial pointing,
nadir tracking, wheel desaturation, and sensor-dropout/fault recovery cases.
The slew must exercise motor saturation. The desaturation case starts above
the mode-entry threshold and must reduce total wheel momentum. The fault case
forces vector/GPS dropout, enters the latched fault mode, resets, and returns
to nominal.

Run the campaign with:

```bash
octave --quiet --eval 'addpath(genpath("matlab")); runIntegratedAdcsCampaign("artifacts/integration");'
```

The CSV records pointing and rate errors, settling time, control effort, wheel
speed/momentum, estimator error, quaternion norm, saturation, and mission
success. PNGs show pointing, estimator convergence, and desaturation.

## Fidelity limits

This is deterministic software/model integration evidence, not hardware or
flight validation. Sensor and actuator values are nominal engineering
parameters. The fixed 50 Hz integration step is adequate for this configured
slow ADCS loop but is not a processor timing measurement. Magnetic unloading
can remove only momentum perpendicular to the instantaneous field and is
therefore assessed over orbital motion rather than at one field orientation.
