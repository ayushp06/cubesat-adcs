# Quaternion Attitude Control

## 1. Intuition

The controller compares where the spacecraft is pointing with where it should
point. That difference becomes a commanded body torque. Reaction wheels create
the torque by accelerating in the opposite direction, exchanging angular
momentum with the spacecraft without applying external torque.

The proportional term acts like a rotational spring: larger attitude error
requests more correcting torque. The derivative term acts like a damper: body
rate produces opposing torque so the spacecraft does not coast through the
target.

## 2. Quaternion attitude error

The repository attitude quaternion `q_IB` rotates body-frame coordinates into
the inertial frame and uses scalar-first Hamilton multiplication. Let `q_ID`
describe the desired body frame D relative to inertial frame I. The error is

\[
q_{DB}=q_{ID}^{*}\otimes q_{IB}.
\]

It rotates the current body frame B into the desired frame D. Unit quaternions
`q` and `-q` describe the same physical attitude, so the implementation negates
the error when its scalar part is negative. This selects the error quaternion
with a nonnegative scalar part and therefore the shortest rotation, avoiding an
unnecessary maneuver greater than 180 degrees.

For a small physical error angle vector \(\delta\theta\),

\[
q_{DB,v}\approx \frac{1}{2}\delta\theta.
\]

For example, if the spacecraft is at identity and the target is +90 degrees
about X, `q_DB` has a negative X vector component. The feedback sign below then
commands positive X body torque toward the target.

## 3. Quaternion PD law

For a stationary reference, the desired body rate is zero. The body-torque
command is

\[
\tau_B=-K_p q_{DB,v}-K_d\omega_{BI}^{B}.
\]

`K_p` and `K_d` are diagonal positive gain matrices. The nominal gains are
derived per principal axis from a natural frequency \(\omega_n\) and damping
ratio \(\zeta\):

\[
K_p=2J\omega_n^2,
\qquad
K_d=2\zeta J\omega_n.
\]

The factor of two in `K_p` compensates for the quaternion small-angle relation.
The current nominal tuning uses \(\omega_n=0.12\) rad/s and \(\zeta=0.9\).
These gains are calibration parameters, not universal flight values; actuator
authority, flexible modes, sensor noise, sampling, and uncertainty must inform
flight tuning.

## 4. Reaction-wheel allocation and limits

The wheel-axis matrix `A` stores each wheel spin axis in body coordinates. A
motor torque vector `u` applies body torque

\[
\tau_B=-A u.
\]

For the current square, full-rank three-wheel assembly, allocation solves

\[
A u=-\tau_B
\]

with MATLAB's backslash operator. No pseudoinverse or null-space optimizer is
needed for the present orthogonal three-wheel geometry.

After allocation, each wheel command is clipped to its motor-torque limit.
Torque that would drive a wheel farther beyond its speed limit is set to zero,
while braking torque remains available. Applied torque can therefore differ
from requested torque; closed-loop metrics use the limited applied value.

## 5. Closed-loop implementation

At every ODE evaluation:

1. `quaternionAttitudeError` computes the normalized shortest-path error.
2. `quaternionPDController` computes desired body torque.
3. `allocateReactionWheelTorque` maps body torque to motor torque.
4. `limitReactionWheelTorque` applies physical command limits.
5. `attitudeDynamics3RW` propagates attitude, body rate, and wheel speed.

Truth attitude and rate are fed directly to the controller in this foundation.
No estimator or sensor model exists yet, so these results validate control
mathematics and ideal actuator coupling rather than flight-like navigation.

## 6. Analytical checks

The automated control test verifies:

- zero error for identical and antipodal representations of one attitude;
- the expected error quaternion for an identity-to-+90-degree-X command;
- restoring proportional-torque sign;
- damping torque opposite to body rate;
- equal-and-opposite torque from wheel allocation.

## 7. Performance metrics

`computeSlewMetrics` reports:

- **pointing error:** shortest quaternion rotation angle to the target;
- **settling time:** first sampled time after which pointing error remains at or
  below 1 degree and body-rate magnitude remains at or below 0.1 degree/s;
- **overshoot:** maximum target-axis rotation beyond the commanded angle;
- **body rate:** peak magnitude and peak absolute component on each axis;
- **control effort:** time integral of absolute applied motor torque per wheel;
- **wheel speed:** peak absolute speed per wheel;
- **wheel momentum:** peak absolute momentum per wheel and peak norm of the
  assembly momentum vector.

Applied, not merely requested, motor torque is used for effort. Overshoot is a
target-axis progress measure; it does not claim that the multi-axis trajectory
follows a unique Euler-angle path.

## 8. Validated slew scenarios

The suite starts from identity attitude, commands a 90-degree shortest-path
rotation, and runs for 180 s. The arbitrary axis is `[1; 2; 3] / sqrt(14)`.
The final case starts with body rate `[1; -0.5; 0.75]` degree/s. GNU Octave
produced the following results on 2026-08-19 with `RelTol=1e-8` and
`AbsTol=1e-10`:

| Case | Final error (deg) | Settling (s) | Overshoot (deg) | Peak rate (deg/s) | Peak wheel speed (rad/s) | Peak total wheel momentum (N m s) |
|---|---:|---:|---:|---:|---:|---:|
| X 90° | < 0.0001 | 52.6 | 0.1114 | 3.7809 | 44.8725 | 0.00224363 |
| Y 90° | < 0.0001 | 51.8 | 0.1128 | 3.8371 | 40.1822 | 0.00200911 |
| Z 90° | < 0.0001 | 48.6 | 0.1169 | 4.0122 | 9.80372 | 0.000490186 |
| Arbitrary axis | < 0.0001 | 48.8 | 0.1167 | 4.0054 | 22.3178 | 0.00135139 |
| Arbitrary + initial rate | < 0.0001 | 59.2 | 0.9189 | 4.3228 | 31.3377 | 0.00160478 |

Per-wheel control effort and momentum are printed by
`runAttitudeSlewSuite.m`. The largest peak wheel speed is only 7.14% of the
configured 628.319 rad/s limit.

A convergence check repeats the arbitrary-axis case at `1e-7/1e-9` and
`1e-9/1e-11` relative/absolute tolerances. At the printed precision, final
attitude difference and settling-time difference were both zero; the automated
test requires final attitude disagreement below `1e-4` degree.

These are deterministic ideal-model results, not hardware performance claims.
Truth state is fed directly to the controller; environmental torques, sensor
errors, estimation, delays, sampling, and flexible dynamics remain future work.
