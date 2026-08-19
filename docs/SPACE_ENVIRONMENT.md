# 6-DOF Truth Model and Space Environment

## 1. Scope and fidelity

This model is a deterministic engineering truth model for low-Earth-orbit ADCS
development. It propagates SI-unit ECI translation and body attitude, then
computes modular environmental forces, torques, and reference vectors. The
models are intentionally transparent and independently testable. They are not
flight-qualification ephemerides or atmosphere/geomagnetic standards.

## 2. Frames, state, and units

- `ECI`: right-handed Earth-centred inertial frame; position is in metres and
  velocity in metres per second.
- `B`: spacecraft body frame defined in `matlab/CONVENTIONS.md`.
- `q_IB`: scalar-first Hamilton quaternion rotating body components into ECI.
- `omega_BI_B`: body angular velocity relative to ECI, expressed in B, in rad/s.
- Forces are newtons, accelerations m/s², and torques N m.

The coupled 13-state truth vector is

\[
x=[r_I^T\;v_I^T\;q_{IB}^T\;\omega_{BI}^{B,T}]^T.
\]

Environment functions accept explicit state and parameter inputs. They do not
read or mutate a hidden global state, so higher-fidelity replacements can be
validated independently.

## 3. Two-body orbit propagation

Point-mass gravity uses

\[
\ddot r_I=-\frac{\mu}{\lVert r_I\rVert^3}r_I,
\]

with `mu = 3.986004418e14 m^3/s^2`. In this conservative model the specific
mechanical energy and specific angular momentum are

\[
\epsilon=\frac{\lVert v\rVert^2}{2}-\frac{\mu}{\lVert r\rVert},
\qquad h=r\times v.
\]

A 400 km circular-orbit validation completed one 5553.624 s orbit with maximum
relative energy error `7.236e-11` and angular-momentum error `3.618e-11` using
`RelTol=1e-10`, `AbsTol=1e-6`.

## 4. J2 oblateness

Earth's equatorial bulge is represented by the first zonal harmonic. With
`s = z/r`, the perturbing acceleration is

\[
a_{J2}=\frac{3J_2\mu R_E^2}{2r^5}
\begin{bmatrix}
x(5s^2-1)\\y(5s^2-1)\\z(5s^2-3)
\end{bmatrix}.
\]

The implementation is checked against closed-form equatorial and polar-axis
values. For the reproducible 400 km, 51.6° scenario, adding J2 changes the
one-orbit endpoint by 72,501.729 m relative to point-mass propagation. This is
a model-to-model perturbation comparison, not position error against truth.

Further sections document the force, torque, illumination, and magnetic models
as they are integrated.
