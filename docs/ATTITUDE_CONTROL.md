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

Closed-loop scenario validation, performance metrics, and solver convergence
are documented with the slew suite once generated.
