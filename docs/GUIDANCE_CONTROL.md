# ADCS Guidance, Control, and Mode Management

## 1. Frames and guidance contract

Guidance consumes estimated navigation/environment inputs and returns the
repository-standard scalar-first Hamilton `q_IB`, plus body-rate reference when
the target moves. Control consumes only estimated attitude/rate and this
reference. Truth remains confined to simulation plant and metric code.

An attitude matrix's columns are the body axes expressed in ECI. Inertial
pointing normalizes and holds a commanded quaternion. Sun/safe pointing aligns
`+Z_B` with the Sun and resolves roll by keeping `+X_B` near inertial `+Z`
(falling back to `+X_I` at the singular geometry). Nadir/LVLH pointing uses

`z_B^I = -r_I/|r_I|`, `x_B^I = v_horizontal/|v_horizontal|`,
`y_B^I = z_B^I x x_B^I`.

## 2. Slew reference

The shortest relative quaternion is converted to axis-angle `(e,theta)`. A
cubic smoothstep `s=3u^2-2u^3`, `u=clamp(t/T,0,1)`, gives zero endpoint rates.
The command is `q_ref=q_0 x q(e,s theta)` and feed-forward body rate is
`omega_ref=e theta (6u-6u^2)/T`. This is a kinematic reference; actuator-aware
duration selection remains the caller's responsibility.

## 3. PD, bounded PID-style control, and LQR

The existing quaternion PD law is retained. Since the vector part of a small
error quaternion is `delta_theta/2`, its effective linear proportional gain is
`Kp/2`; rate feedback is `-Kd omega`.

The PID-style comparison integrates the physical small-angle error, clamps the
integral to 20 degrees per axis, and freezes it while torque is saturated.
This is technically appropriate integral augmentation for removing constant
disturbance bias without allowing wheel-limited maneuvers to wind up.

Near zero error/rate, `x=[delta_theta;omega]` obeys
`xdot = A x + B tau`, with `A=[0 I;0 0]` and `B=[0;J^-1]`. The LQR minimizes
`integral(x'Qx + tau'Rtau)dt`. `continuousLqr` solves the continuous algebraic
Riccati equation from the Hamilton matrix stable subspace, avoiding an external
control-toolbox dependency. The implementation records controllability rank,
closed-loop eigenvalues, and Riccati matrix; tests require rank six, stable
eigenvalues, and positive-definite symmetric cost-to-go.

`runControllerComparison` applies identical component torque saturation and a
90-degree slew to PD, bounded PID, and LQR, reporting final error, settling
time, peak rate, and integrated torque magnitude.

GNU Octave on 2026-08-19 measured: PD `52.60 s` settling and `4.4935e-3 N m s`
effort; PID `52.60 s`, `0.0480 deg` final error, and `4.4957e-3 N m s`; LQR
`63.40 s` and `5.3604e-3 N m s`. These compare the documented nominal tuning,
not universally optimal controller performance.

## 4. Magnetic actuation, detumble, and wheel unloading

Three orthogonal rods produce body dipole `m_B=A_m m_rods`, with each rod
limited to 0.2 A m^2. Magnetic torque is `tau_B=m_B x B_B`, so torque parallel
to the local field is impossible.

B-dot uses `m_cmd=-k_Bdot (B[k]-B[k-1])/dt`. For a body rotating in a slowly
changing field, `Bdot approximately B x omega`; therefore the resulting torque
opposes the component of angular rate perpendicular to the field. Orbital field
direction variation supplies three-axis damping over time. The 10,000 s
validation uses a 5,400 s changing LEO-scale field and confirms an initial
`[8,-6,5] deg/s` tumble falls below 1 deg/s despite dipole saturation.

For unloading, desired external torque is `-k_h h_w`. The minimum-norm dipole
`m=B x tau_des/|B|^2` realizes the field-perpendicular projection. After rod
saturation, the achieved magnetic torque—not the unattainable request—is
mapped to wheel motor torque. Reaction-wheel body torque and magnetic torque
then cancel ideally while wheel momentum decreases. Tests require momentum to
fall below 20% of its initial value without exceeding wheel/rod limits.
