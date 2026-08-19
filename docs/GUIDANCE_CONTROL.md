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
