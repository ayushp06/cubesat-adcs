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

## 5. Gravity-gradient torque

With `n_B = C_IB^T r_I/|r_I|`, the point-mass result is
`tau_gg_B = (3 mu/r^3) n_B x (J n_B)`. It is zero when the local vertical is
a principal inertia axis. A 45-degree case is checked against its closed-form
torque and sign.

## 6. Atmosphere, drag, and aerodynamic torque

Density uses the configurable model `rho = rho0 exp(-(h-h0)/H)`, nominally
anchored at 400 km. The atmosphere rigidly co-rotates with Earth, so
`v_rel = v_ECI - omega_E x r_ECI`. Drag is
`F_D = -0.5 rho C_D A_p |v_rel| v_rel`, where `A_p` is the projected area of
the configured rectangular box. Torque is `r_CP x F_D` in B. Space weather,
winds, and free-molecular details are omitted; the explicit interface permits
a later high-fidelity atmosphere replacement.

## 7. Sun, eclipse, and solar radiation pressure

The Sun follows a circular annual ephemeris in the mean ecliptic, rotated by
the configured obliquity into ECI. Distance is fixed at one AU. Eclipse is a
binary cylindrical shadow: behind Earth and within one Earth radius of the
Sun-Earth axis is dark. Penumbra is omitted.

Solar pressure is `F_SRP = -nu P0 C_R A_p s_hat`. The box projection changes
with attitude and the configured centre-of-pressure gives `r_CP x F_SRP`.
Face-level optical properties are collapsed into one reflectivity coefficient.

## 8. Geomagnetic field and torque

The centered dipole is tilted 9.3 degrees from Earth's spin axis and fixed in
ECEF. It rotates into ECI at the Earth rate and gives
`B_I = 1e-7/r^3 [3 r_hat (m_I dot r_hat) - m_I]` in tesla. Disturbance torque
is `m_B x B_B` for the configurable residual dipole. Secular variation, higher
harmonics, and external fields are omitted; use IGRF when geographic accuracy
is required.

## 9. Reproducible module validation

Run `octave --quiet matlab/tests/testSpaceEnvironment.m` from the repository
root. It checks gravity-gradient limiting cases, density, drag, Sun/eclipse
geometry, SRP shadowing, and dipole field/torque relations.

## 10. Coupled 13-state propagation

`fullSpacecraftDynamics` assembles the independent modules without hidden
state. Its derivative is `[v_I; a_I; qdot_IB; omegadot_BI_B]`. Translation
includes two-body gravity and configurable J2, drag, and solar pressure.
Rotation uses Euler rigid-body dynamics with configurable gravity-gradient,
aerodynamic, solar-pressure, and residual-magnetic torques. Translation and
rotation couple only through physical force/torque inputs and attitude-based
projected area; no estimator or flight state is present.

`truthModelParams` exposes effect switches for analytical isolation and
regression testing. These are fidelity switches, not hidden physical values;
all constants remain in the Earth and spacecraft parameter structures.

Run `testFullSpacecraftDynamics` from `matlab/tests` for the conservative
one-orbit invariant check and all-effects derivative check. Run
`run6DOFValidation` from `matlab/simulations` for the reproducible 400 km,
51.6-degree, all-effects orbit. On 2026-08-19 GNU Octave completed 5553.624 s,
ending at 399.975 km altitude with maximum quaternion norm error `1.336e-08`.
That altitude is a model result, not an accuracy claim against ephemeris data.
