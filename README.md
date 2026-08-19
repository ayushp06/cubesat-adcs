# CubeSat ADCS Digital Twin

## A beginner-to-research-grade guide to the repository as it exists today

This repository is the beginning of an **Attitude Determination and Control System (ADCS) digital twin** for a nominal 3U CubeSat. The code currently models attitude representation, torque-free rigid-body rotation, and angular-momentum exchange with one or three reaction wheels. It does **not yet** model orbit propagation, sensors, attitude estimation, environmental torques, guidance, feedback control, motor electrical dynamics, or momentum dumping.

This document explains every repository file, derives the mathematics used by the code, shows how information flows through each simulation, records what has been verified, and separates implemented behavior from future intent.

> Status snapshot: project status and current validation results are maintained in [`docs/STATUS.md`](docs/STATUS.md).

---

## 1. What problem is this project solving?

A satellite must know and control two different kinds of motion:

1. **Translation**: where its center of mass is in orbit.
2. **Rotation**: how the satellite is oriented and how quickly it is turning.

This repository currently addresses rotation. A spacecraft may need to point a camera at Earth, aim an antenna toward a ground station, keep solar panels illuminated, or avoid an unsafe tumble. ADCS is the spacecraft subsystem responsible for determining and controlling that orientation.

A complete ADCS generally forms this loop:

```text
environment -> sensors -> attitude estimator -> guidance -> controller
       ^                                                |
       |                                                v
       +-------- spacecraft dynamics <- actuators <-----+
```

The present repository implements only the mathematical foundation and part of the right-hand side of this loop:

```text
prescribed torque -> reaction-wheel/spacecraft dynamics -> attitude and rates
```

That is a sensible first milestone. A controller cannot be trusted until the uncontrolled plant model and its coordinate conventions are correct.

### 1.1 Essential vocabulary

| Term | Meaning |
|---|---|
| Attitude | Orientation of the spacecraft relative to a reference frame. |
| Angular velocity | Rate and axis of rotation, in radians per second. |
| Torque | Rotational equivalent of force, in newton-metres. |
| Inertia tensor | Matrix describing resistance to angular acceleration about each axis. |
| Quaternion | Four-number, singularity-free representation of a 3-D rotation. |
| Direction cosine matrix (DCM) | A 3-by-3 matrix that rotates vector coordinates between frames. |
| Reaction wheel | Motor-driven flywheel that exchanges angular momentum with the spacecraft. |
| State | Minimum set of variables propagated by the differential equation solver. |
| Digital twin | A computational model intended to reproduce relevant behavior of a physical system. The current code is an early dynamics model, not yet a calibrated full twin. |

---

## 2. Repository map and the purpose of every file

```text
matlab/
├── CONVENTIONS.md
├── config/
│   ├── reactionWheelParams.m
│   ├── spacecraftParams.m
│   └── spacecraftParams.asv
├── dynamics/
│   ├── attitudeDynamics.m
│   ├── attitudeDynamicsRW.m
│   └── attitudeDynamics3RW.m
├── math/
│   ├── .m
│   ├── CubeSat_ADCS_Quaternion_Math_Explained.pdf
│   ├── quatConjugate.m
│   ├── quatMultiply.m
│   ├── quatNormalize.m
│   └── quatToDCM.m
├── simulations/
│   ├── runTorqueFreeRotation.m
│   ├── runReactionWheelTest.m
│   └── runThreeWheelTest.m
└── tests/
    └── testQuaternionMath.m
```

| File | What it contributes | Current status |
|---|---|---|
| `matlab/CONVENTIONS.md` | Declares frame, quaternion, angular-rate, unit, and angle conventions. | Authoritative and essential. |
| `matlab/config/spacecraftParams.m` | Returns nominal name, mass, dimensions, and inertia tensor. | Used by all simulations. |
| `matlab/config/spacecraftParams.asv` | MATLAB Editor autosave copy of `spacecraftParams.m`. | Byte-for-byte duplicate; not used at runtime. |
| `matlab/config/reactionWheelParams.m` | Returns three wheel inertias, torque limits, speed limits, and wheel-axis matrix. | Shared by the three-wheel model and the one-wheel model's first-wheel selection. |
| `matlab/math/quatMultiply.m` | Implements scalar-first Hamilton multiplication. | Tested. |
| `matlab/math/quatConjugate.m` | Negates the quaternion vector part. | Tested as the inverse of a unit quaternion. |
| `matlab/math/quatNormalize.m` | Projects a quaternion to unit length and rejects near-zero input. | Tested. |
| `matlab/math/quatToDCM.m` | Produces the body-to-inertial rotation matrix. | Tested for identity, orthogonality, determinant, and a known rotation. |
| `matlab/math/CubeSat_ADCS_Quaternion_Math_Explained.pdf` | Six-page earlier explanation of quaternion purpose, operations, DCMs, and tests. | Consistent with the live convention; narrower than this guide. |
| `matlab/math/.m` | Empty file. | No runtime or documentation effect. |
| `matlab/dynamics/attitudeDynamics.m` | Seven-state rigid-body attitude and rate differential equation. | Used by torque-free simulation. |
| `matlab/dynamics/attitudeDynamicsRW.m` | Eight-state spacecraft plus wheel 1 on the X axis. | Uses the first entry of the three-wheel parameter set. |
| `matlab/dynamics/attitudeDynamics3RW.m` | Ten-state coupled spacecraft plus three orthogonal wheels model. | Used by three-wheel simulation. |
| `matlab/simulations/runTorqueFreeRotation.m` | Runs 100 s of torque-free principal-axis rotation and plots invariants. | Runs in Octave. |
| `matlab/simulations/runReactionWheelTest.m` | Demonstrates one-wheel momentum exchange for 10 s. | Runs in Octave. |
| `matlab/simulations/runThreeWheelTest.m` | Applies constant commands to three wheels for 10 s and plots body/wheel rates. | Runs in Octave. |
| `matlab/tests/testQuaternionMath.m` | Seven assertion-based checks of quaternion utilities and conventions. | All pass in Octave. |
| `matlab/tests/testReactionWheelDynamics.m` | Checks one-wheel analytical acceleration and momentum conservation. | All pass in Octave. |

The Git history shows four development stages: the quaternion mathematics and conventions, relocation of the test, addition of torque-free and one-wheel dynamics, and finally addition of the three-wheel model.

---

## 3. Conventions: the contract that makes the math meaningful

Three-dimensional rotation formulas are convention-dependent. Two individually valid sources can appear to disagree because they use different quaternion orderings, multiplication rules, frame directions, or active/passive interpretations. `CONVENTIONS.md` removes this ambiguity.

### 3.1 Frames

- (I): Earth-centered inertial frame.
- (B): spacecraft body-fixed frame.
- Both frames are right-handed.

The body frame rotates with the spacecraft. A sensor mounted to the spacecraft naturally reports components in (B). An orbit or environment model often provides vectors in (I).

### 3.2 Quaternion definition

The attitude is

\[
\mathbf q_{IB}=
\begin{bmatrix}q_w&q_x&q_y&q_z\end{bmatrix}^T,
\]

using:

- scalar-first ordering;
- Hamilton multiplication;
- a body-to-inertial meaning.

Thus the corresponding DCM obeys

\[
\mathbf v_I=\mathbf C_{IB}\mathbf v_B.
\]

The inverse transformation is

\[
\mathbf v_B=\mathbf C_{IB}^T\mathbf v_I,
\]

because a proper rotation matrix is orthogonal.

### 3.3 Angular velocity

The propagated rate is

\[
\boldsymbol\omega_{BI}^{B},
\]

meaning the angular velocity of body (B) relative to inertial frame (I), expressed in body coordinates. The superscript matters: Euler's rotational equation takes its simplest familiar form when the components are expressed in the rotating body frame.

### 3.4 Units

All internal calculations use SI:

- metres, kilograms, seconds;
- radians and radians per second;
- torque in N m;
- angular momentum in N m s, equivalently kg m²/s;
- inertia in kg m².

Degrees are only for human input or plotting. MATLAB's `deg2rad` and `rad2deg` mark those boundaries.

---

## 4. Spacecraft and actuator configuration

### 4.1 Nominal spacecraft

`spacecraftParams.m` returns:

\[
m=4.0\ \text{kg}, \qquad
\mathbf d=\begin{bmatrix}0.10&0.10&0.30\end{bmatrix}^T\ \text{m},
\]

and the diagonal inertia tensor

\[
\mathbf J=
\begin{bmatrix}
0.034&0&0\\
0&0.030&0\\
0&0&0.007
\end{bmatrix}\ \text{kg m}^2.
\]

For a uniform rectangular box with side lengths \(a,b,c\), the centroidal principal moments are

\[
J_x=\frac{m}{12}(b^2+c^2),\quad
J_y=\frac{m}{12}(a^2+c^2),\quad
J_z=\frac{m}{12}(a^2+b^2).
\]

Using 0.10 m, 0.10 m, and 0.30 m gives approximately

\[
\operatorname{diag}(0.0333,0.0333,0.00667)\ \text{kg m}^2.
\]

The configured \((0.034,0.030,0.007)\) values are therefore plausible approximations for a nearly box-shaped 3U spacecraft with some nonuniform mass distribution. They are not derived from a component-level mass model in this repository.

The off-diagonal products of inertia are zero. This asserts that \(x,y,z\) are principal axes. If the real mass distribution is not aligned with those axes, the measured tensor should be symmetric but not diagonal.

### 4.2 Reaction wheels

Each of the three configured wheels has

\[
J_{w,i}=5\times10^{-5}\ \text{kg m}^2,
\qquad |u_i|\le2\times10^{-4}\ \text{N m}.
\]

The speed limit is entered as 6000 RPM and converted by

\[
6000\frac{\text{rev}}{\text{min}}
\left(\frac{2\pi\ \text{rad}}{1\ \text{rev}}\right)
\left(\frac{1\ \text{min}}{60\ \text{s}}\right)
=200\pi\approx628.319\ \text{rad/s}.
\]

The wheel-axis matrix is

\[
\mathbf A=\mathbf I_3.
\]

Its columns are wheel spin-axis unit vectors expressed in body coordinates. Therefore wheel 1 lies on (+x_B), wheel 2 on (+y_B), and wheel 3 on (+z_B). A general assembly could use non-axis-aligned columns, including a four-wheel pyramidal layout, but the current code and parameter set contain exactly three orthogonal wheels.

The maximum stored momentum per wheel implied by the configured limit is

\[
h_{w,\max}=J_w\Omega_{\max}
=(5\times10^{-5})(628.319)
\approx0.03142\ \text{N m s}.
\]

This speed limit is stored but never enforced by the dynamics. Torque is clipped; speed is not.

---

## 5. Quaternion mathematics from first principles

### 5.1 Why attitude needs more than three ordinary coordinates

A 3-D orientation has three degrees of freedom, but common three-angle descriptions develop singularities. Quaternions use four numbers plus one constraint. The redundancy avoids gimbal lock and makes composition and numerical propagation convenient.

A valid attitude quaternion has unit norm:

\[
\|\mathbf q\|^2=q_w^2+q_x^2+q_y^2+q_z^2=1.
\]

A rotation by angle \(\theta\) about unit axis

\[
\hat{\mathbf e}=\begin{bmatrix}e_x&e_y&e_z\end{bmatrix}^T
\]

is represented by

\[
\mathbf q=
\begin{bmatrix}
\cos(\theta/2)\\
\hat{\mathbf e}\sin(\theta/2)
\end{bmatrix}.
\]

The half-angle is a property of the mapping from unit quaternions to physical rotations. It also creates the double-cover property: \(\mathbf q\) and \(-\mathbf q\) represent the same physical attitude.

### 5.2 Hamilton multiplication: `quatMultiply.m`

Write each quaternion as a scalar and vector:

\[
\mathbf q_1=\begin{bmatrix}s_1\\\mathbf v_1\end{bmatrix},\qquad
\mathbf q_2=\begin{bmatrix}s_2\\\mathbf v_2\end{bmatrix}.
\]

Their Hamilton product is

\[
\mathbf q_1\otimes\mathbf q_2=
\begin{bmatrix}
s_1s_2-\mathbf v_1^T\mathbf v_2\\
s_1\mathbf v_2+s_2\mathbf v_1+\mathbf v_1\times\mathbf v_2
\end{bmatrix}.
\]

The implementation is a direct transcription of this equation using `dot` and `cross`. Quaternion multiplication is associative but generally not commutative:

\[
(\mathbf q_1\otimes\mathbf q_2)\otimes\mathbf q_3
=\mathbf q_1\otimes(\mathbf q_2\otimes\mathbf q_3),
\]

but usually

\[
\mathbf q_1\otimes\mathbf q_2\ne\mathbf q_2\otimes\mathbf q_1.
\]

That is physically necessary: rotate 90° about (x) and then 90° about (y), and the result differs from doing them in reverse order.

### 5.3 Conjugation and inversion: `quatConjugate.m`

For

\[
\mathbf q=\begin{bmatrix}q_w&\mathbf q_v^T\end{bmatrix}^T,
\]

the conjugate is

\[
\mathbf q^*=\begin{bmatrix}q_w&-\mathbf q_v^T\end{bmatrix}^T.
\]

The general inverse is

\[
\mathbf q^{-1}=\frac{\mathbf q^*}{\|\mathbf q\|^2}.
\]

For a unit attitude quaternion, this reduces to \(\mathbf q^{-1}=\mathbf q^*\). The code provides the conjugate, not a general inverse routine. Its use as an inverse is valid only when the input has unit length.

### 5.4 Normalization: `quatNormalize.m`

Numerical integration uses finite precision, so the unit constraint can drift. The function computes

\[
\mathbf q_{\text{unit}}=\frac{\mathbf q}{\|\mathbf q\|}.
\]

It rejects norms smaller than MATLAB's `eps`. This protects against division by an effectively zero magnitude, but it does not validate input shape, finite values, or complex values.

Normalization projects a point back to the unit 3-sphere \(S^3\). The simulations normalize stored output for plotting in the torque-free and one-wheel scripts. The differential equation itself does not renormalize during each `ode45` evaluation.

### 5.5 Quaternion to DCM: `quatToDCM.m`

After normalizing its input, the function constructs

\[
\mathbf C_{IB}=
\begin{bmatrix}
1-2(q_y^2+q_z^2) & 2(q_xq_y-q_wq_z) & 2(q_xq_z+q_wq_y)\\
2(q_xq_y+q_wq_z) & 1-2(q_x^2+q_z^2) & 2(q_yq_z-q_wq_x)\\
2(q_xq_z-q_wq_y) & 2(q_yq_z+q_wq_x) & 1-2(q_x^2+q_y^2)
\end{bmatrix}.
\]

A valid DCM satisfies

\[
\mathbf C^T\mathbf C=\mathbf I,\qquad
\det(\mathbf C)=+1.
\]

The first condition preserves lengths and angles. The second excludes reflections. For example, a +90° rotation about (+z) has

\[
\mathbf q=
\begin{bmatrix}\cos45^\circ&0&0&\sin45^\circ\end{bmatrix}^T
\]

and maps

\[
\begin{bmatrix}1\\0\\0\end{bmatrix}_B
\mapsto
\begin{bmatrix}0\\1\\0\end{bmatrix}_I.
\]

This known-answer example tests the complete convention rather than merely testing matrix algebra.

---

## 6. Quaternion kinematics: how angular velocity changes attitude

All three dynamics functions use

\[
\dot{\mathbf q}_{IB}
=\frac12\mathbf q_{IB}\otimes
\begin{bmatrix}0\\\boldsymbol\omega_{BI}^{B}\end{bmatrix}.
\]

The angular velocity is embedded as a **pure quaternion** whose scalar part is zero. Expanding the product gives

\[
\dot q_w=-\frac12\mathbf q_v^T\boldsymbol\omega,
\]

\[
\dot{\mathbf q}_v
=\frac12\left(q_w\boldsymbol\omega+\mathbf q_v\times\boldsymbol\omega\right).
\]

Equivalently,

\[
\dot{\mathbf q}
=\frac12
\begin{bmatrix}
-q_x&-q_y&-q_z\\
q_w&-q_z&q_y\\
q_z&q_w&-q_x\\
-q_y&q_x&q_w
\end{bmatrix}
\boldsymbol\omega.
\]

Why the factor \(1/2\)? Because the quaternion uses the physical half-angle. For constant principal-axis rotation about \(x\), starting at identity,

\[
\mathbf q(t)=
\begin{bmatrix}
\cos(\omega_xt/2)\\
\sin(\omega_xt/2)\\
0\\0
\end{bmatrix}.
\]

The code's multiplication order and this analytic result are consistent with the declared body-to-inertial Hamilton convention.

---

## 7. Rigid-body rotational dynamics

### 7.1 From Newton's law to Euler's equation

For translation, Newton's second law is \(\mathbf F=m\mathbf a\). For rotation, the inertial derivative of angular momentum equals external torque:

\[
\left(\frac{d\mathbf H}{dt}\right)_I=\boldsymbol\tau_{\text{ext}}.
\]

The code expresses vectors in rotating body coordinates. The transport theorem relates inertial and body derivatives:

\[
\left(\frac{d\mathbf H}{dt}\right)_I
=\left(\frac{d\mathbf H}{dt}\right)_B
+\boldsymbol\omega\times\mathbf H.
\]

For a rigid body, \(\mathbf H=\mathbf J\boldsymbol\omega\), and \(\mathbf J\) is constant in body coordinates. Therefore

\[
\boldsymbol\tau
=\mathbf J\dot{\boldsymbol\omega}
+\boldsymbol\omega\times(\mathbf J\boldsymbol\omega),
\]

so

\[
\dot{\boldsymbol\omega}
=\mathbf J^{-1}
\left[\boldsymbol\tau
-\boldsymbol\omega\times(\mathbf J\boldsymbol\omega)\right].
\]

The cross-product term is the gyroscopic coupling caused by expressing momentum in a rotating frame. It is not an optional correction.

MATLAB computes this with

```matlab
angularMomentum = J * omega;
gyroscopicTerm = cross(omega, angularMomentum);
omegaDot = J \ (tau - gyroscopicTerm);
```

The backslash operator solves \(\mathbf J\dot{\boldsymbol\omega}=\cdots\) without explicitly forming \(\mathbf J^{-1}\), which is the standard and numerically preferable operation.

### 7.2 Component form for principal axes

With diagonal \(\mathbf J=\operatorname{diag}(J_x,J_y,J_z)\), Euler's equations are

\[
J_x\dot\omega_x+(J_z-J_y)\omega_y\omega_z=\tau_x,
\]

\[
J_y\dot\omega_y+(J_x-J_z)\omega_z\omega_x=\tau_y,
\]

\[
J_z\dot\omega_z+(J_y-J_x)\omega_x\omega_y=\tau_z.
\]

Rotation exactly about a principal axis makes the products of the other two rates zero. With zero torque, that rate remains constant. General torque-free motion can still have time-varying body-frame components even though inertial angular momentum is constant.

### 7.3 Conserved quantities for a torque-free rigid body

With \(\boldsymbol\tau=0\), two useful invariants are:

\[
T=\frac12\boldsymbol\omega^T\mathbf J\boldsymbol\omega
\quad\text{(rotational kinetic energy)},
\]

and

\[
\|\mathbf H\|=\|\mathbf J\boldsymbol\omega\|
\quad\text{(angular-momentum magnitude)}.
\]

These are stronger validation signals than a plot that merely looks smooth.

---

## 8. Reaction-wheel physics

### 8.1 How an internal actuator rotates a spacecraft

A reaction wheel is a rotor attached to the spacecraft. If its motor accelerates the wheel in one direction, the motor applies an equal and opposite torque to the spacecraft. With no external torque, total angular momentum is conserved.

For one ideal wheel,

\[
h_w=J_w\Omega,
\qquad
\dot\Omega=\frac{u}{J_w},
\]

where (u) is torque applied to the wheel. The body receives (-u) along the same axis.

At the configured values, a (10^{-4}\) N m command produces

\[
\dot\Omega=\frac{10^{-4}}{5\times10^{-5}}=2\ \text{rad/s}^2.
\]

Starting from rest, the ideal wheel reaches 20 rad/s after 10 s. If the spacecraft begins at rest and gyroscopic coupling is absent for pure (x)-axis motion,

\[
\dot\omega_x=-\frac{10^{-4}}{0.034}
\approx-2.94118\times10^{-3}\ \text{rad/s}^2,
\]

so after 10 s

\[
\omega_x\approx-0.0294118\ \text{rad/s}.
\]

The momenta cancel:

\[
J_x\omega_x+J_w\Omega
=(0.034)(-0.0294118)+(5\times10^{-5})(20)\approx0.
\]

### 8.2 Three-wheel assembly

Let

\[
\boldsymbol\Omega=
\begin{bmatrix}\Omega_1&\Omega_2&\Omega_3\end{bmatrix}^T,
\quad
\mathbf J_w=\operatorname{diag}(J_{w,1},J_{w,2},J_{w,3}),
\]

and let \(\mathbf A\) contain the wheel axes as columns. Wheel momentum in body coordinates is

\[
\mathbf H_w^B=\mathbf A\mathbf J_w\boldsymbol\Omega.
\]

The total internal angular momentum represented by `attitudeDynamics3RW.m` is

\[
\mathbf H_{\text{total}}^B
=\mathbf J\boldsymbol\omega+\mathbf A\mathbf J_w\boldsymbol\Omega.
\]

The wheel equations are element-wise because the configuration stores inertia as a vector:

\[
\dot{\boldsymbol\Omega}=\mathbf J_w^{-1}\mathbf u.
\]

The body reaction torque is

\[
\boldsymbol\tau_B=-\mathbf A\mathbf u.
\]

The code propagates

\[
\dot{\boldsymbol\omega}
=\mathbf J^{-1}
\left[-\mathbf A\mathbf u
-\boldsymbol\omega\times
(\mathbf J\boldsymbol\omega+\mathbf H_w^B)\right].
\]

This wheel-momentum term matters: a spinning rotor contributes gyroscopic coupling even when its motor torque is zero.

Combining the body and wheel equations gives

\[
\left(\frac{d\mathbf H_{\text{total}}}{dt}\right)_B
=-\boldsymbol\omega\times\mathbf H_{\text{total}},
\]

which means the inertial derivative is zero. Thus the ideal model conserves total inertial angular momentum under purely internal motor torque.

### 8.3 Torque saturation

Both wheel dynamics functions clip commands using element-wise `min` and `max`:

\[
u_i\leftarrow\max(\min(u_i,u_{i,\max}),-u_{i,\max}).
\]

This represents a symmetric motor torque limit. It is an instantaneous hard saturation; the model includes no voltage, current, thermal, speed-dependent torque curve, friction, or control electronics.

---

## 9. State vectors and function interfaces

### 9.1 Bare spacecraft: `attitudeDynamics.m`

The seven-state vector is

\[
\mathbf x=
\begin{bmatrix}
\mathbf q_{IB}\\
\boldsymbol\omega_{BI}^{B}
\end{bmatrix}
\in\mathbb R^7.
\]

The function returns

\[
\dot{\mathbf x}=
\begin{bmatrix}
\dot{\mathbf q}_{IB}\\
\dot{\boldsymbol\omega}_{BI}^{B}
\end{bmatrix}.
\]

Its first argument is written `~` because MATLAB ODE solvers require a time argument, but the present dynamics are time-invariant.

### 9.2 One wheel: `attitudeDynamicsRW.m`

The intended eight-state vector is

\[
\mathbf x=
\begin{bmatrix}
\mathbf q_{IB}\\
\boldsymbol\omega_{BI}^{B}\\
\Omega_x
\end{bmatrix}
\in\mathbb R^8.
\]

The implementation reads `wheelSpeed` but never uses it because it omits wheel angular momentum from the gyroscopic term. That omission is harmless for the script's ideal, perfectly aligned, pure (x)-axis case, but it is not a general coupled wheel model.

### 9.3 Three wheels: `attitudeDynamics3RW.m`

The ten-state vector is

\[
\mathbf x=
\begin{bmatrix}
\mathbf q_{IB}\\
\boldsymbol\omega_{BI}^{B}\\
\boldsymbol\Omega
\end{bmatrix}
\in\mathbb R^{10}.
\]

Unlike the one-wheel function, this version includes wheel momentum in total body-frame momentum and supports a general 3-by-3 wheel-axis matrix, provided the three wheel parameter vectors remain dimensionally compatible.

---

## 10. Numerical integration with `ode45`

The scripts call MATLAB/Octave's `ode45`, an adaptive explicit Runge-Kutta method suitable for many nonstiff ordinary differential equations. Conceptually, the solver repeatedly:

1. receives the current time \(t\) and state \(\mathbf x(t)\);
2. asks a dynamics function for \(\dot{\mathbf x}=f(t,\mathbf x)\);
3. estimates the state at a later time using several derivative evaluations;
4. estimates local numerical error;
5. accepts or rejects the step and adapts its size.

The anonymous function captures constant parameters:

```matlab
dynamicsFcn = @(t,x) attitudeDynamics(t, x, sc, tau);
[t, x] = ode45(dynamicsFcn, tspan, x0);
```

This converts a five-input project function into the two-input interface expected by `ode45`.

No `odeset` tolerances or maximum step are specified, so solver defaults apply. Research comparisons should record explicit tolerances and perform convergence studies rather than treating one default-tolerance trajectory as ground truth.

---

## 11. Simulation walkthroughs

### 11.1 Torque-free rotation: `runTorqueFreeRotation.m`

Purpose: validate the baseline seven-state rigid-body and quaternion propagation without actuator or environmental torque.

Step by step:

1. Clear the workspace, console, and figures.
2. Add configuration, math, dynamics, and test directories to MATLAB's path.
3. Load the nominal spacecraft.
4. Set identity attitude ([1,0,0,0]^T).
5. Set initial rate to \(5^\circ/s\) about \(+x_B\), converted to
   \[
   \omega_x=5\pi/180\approx0.0872665\ \text{rad/s}.
   \]
6. Set applied torque to zero.
7. Integrate for 100 s.
8. Normalize the saved quaternion samples.
9. Plot rates and quaternion components.
10. Plot quaternion norm, kinetic energy, relative energy error, momentum magnitude, and relative momentum error.

Because (x) is a principal axis and the other rates begin at zero, the exact rate remains constant. The attitude has the analytic solution

\[
\mathbf q(t)=
\begin{bmatrix}
\cos(0.0872665t/2)\\
\sin(0.0872665t/2)\\0\\0
\end{bmatrix}.
\]

After 100 s the physical rotation is \(500^\circ\). The quaternion half-angle is \(250^\circ\), giving approximately

\[
\mathbf q(100)=
\begin{bmatrix}-0.342020&-0.939693&0&0\end{bmatrix}^T.
\]

The Octave run produced \([-0.342011,-0.939696,0,0]^T\) and constant \(\omega_x=0.0872665\) rad/s, consistent with the analytic result within default integration accuracy. After output normalization, the reported maximum quaternion norm error was \(2.22\times10^{-16}\).

The initial kinetic energy is

\[
T=\frac12(0.034)(0.0872665)^2
\approx1.2946\times10^{-4}\ \text{J},
\]

and momentum magnitude is

\[
H=(0.034)(0.0872665)
\approx2.9671\times10^{-3}\ \text{N m s}.
\]

Important interpretation: this example validates principal-axis propagation, not the more demanding general torque-free tumbling case. With two or three nonzero initial rate components, the body rates should exchange through Euler coupling while energy and inertial momentum remain constant.

### 11.2 One-wheel test: `runReactionWheelTest.m`

Intended purpose: demonstrate equal-and-opposite angular momentum exchange between a stationary spacecraft and one (x)-axis wheel under a constant (10^{-4}) N m command for 10 s.

The script is designed to:

1. create the eight-state initial condition;
2. integrate `attitudeDynamicsRW`;
3. normalize attitude output;
4. plot spacecraft and wheel speeds;
5. compute
   \[
   H_x=J_x\omega_x+J_w\Omega;
   \]
6. report the maximum deviation from the initially zero momentum.

The one-wheel model explicitly uses `rw.J(1)` and `rw.maxTorque(1)`, preserving
its scalar X-axis interface while sharing the three-wheel parameter source. Its
automated regression test checks the analytical accelerations, 20 rad/s final
wheel speed after 10 s, and conservation of X-axis angular momentum.

### 11.3 Three-wheel test: `runThreeWheelTest.m`

Purpose: exercise the ten-state coupled model with simultaneous commands on all three orthogonal wheels.

Initial state:

\[
\mathbf q_0=\begin{bmatrix}1&0&0&0\end{bmatrix}^T,
\quad
\boldsymbol\omega_0=\mathbf0,
\quad
\boldsymbol\Omega_0=\mathbf0.
\]

Commands:

\[
\mathbf u=
\begin{bmatrix}
1.0\times10^{-4}\\
-5.0\times10^{-5}\\
8.0\times10^{-5}
\end{bmatrix}\ \text{N m}.
\]

All are below the configured \(2\times10^{-4}\) N m saturation. Because every wheel inertia is \(5\times10^{-5}\) kg m²,

\[
\dot{\boldsymbol\Omega}
=\begin{bmatrix}2&-1&1.6\end{bmatrix}^T\ \text{rad/s}^2.
\]

After 10 s, the exact ideal wheel speeds are

\[
\boldsymbol\Omega(10)
=\begin{bmatrix}20&-10&16\end{bmatrix}^T\ \text{rad/s},
\]

which the Octave run reproduced exactly to displayed precision. Its final body rate was approximately

\[
\boldsymbol\omega(10)=
\begin{bmatrix}
-0.0294118\\0.0166667\\-0.114286
\end{bmatrix}\ \text{rad/s},
\]

and final quaternion approximately

\[
\mathbf q(10)=
\begin{bmatrix}
0.955940&-0.0724463&0.0410529&-0.281506
\end{bmatrix}^T.
\]

The component-wise final body rates also equal the simple momentum balance

\[
\omega_i=-\frac{J_w\Omega_i}{J_i},
\]

because total momentum began at zero. The gyroscopic term is identically zero when total body-frame momentum is zero, even though the individual body and wheel momenta are nonzero.

The script plots body rates and wheel speeds. It does not normalize saved quaternions, plot attitude, verify quaternion norm, or explicitly calculate total momentum.

---

## 12. Test suite explained

`testQuaternionMath.m` uses a strict tolerance of \(10^{-12}\) and seven assertions:

1. **Identity multiplication** checks \(\mathbf q_I\otimes\mathbf q_I=\mathbf q_I\), where \(\mathbf q_I=[1,0,0,0]^T\).
2. **Normalization** checks that \([2,0,0,0]^T\) becomes the identity unit quaternion.
3. **Conjugate/inverse** checks \(\mathbf q\otimes\mathbf q^*=\mathbf q_I\) for a normalized nontrivial quaternion.
4. **Identity DCM** checks that identity attitude produces \(\mathbf I_3\).
5. **DCM orthogonality** checks \(\mathbf C\mathbf C^T=\mathbf I_3\) in Frobenius norm.
6. **DCM determinant** checks \(\det\mathbf C=1\).
7. **Known +90° (z) rotation** checks the full convention by mapping (+x_B) to (+y_I).

Together these test core algebra and the selected frame convention. They do not currently test:

- invalid quaternion inputs;
- multiplication composition order using two different rotations;
- bare-spacecraft and three-wheel ODE invariants;
- torque saturation;
- wheel-axis allocation;
- speed saturation;
- energy or momentum with automated assertions;
- MATLAB-versus-Octave parity.

On the documented snapshot, all seven existing assertions pass in GNU Octave.

---

## 13. How to run the repository

Run scripts from their own directories because the relative `addpath("../...")` calls are resolved from the current working directory.

### MATLAB

```matlab
cd matlab/tests
testQuaternionMath
testReactionWheelDynamics

cd ../simulations
runTorqueFreeRotation
runThreeWheelTest
```

`testReactionWheelDynamics` also validates the one-wheel model analytically.

### GNU Octave from the repository root

```sh
octave --quiet --eval "cd('matlab/tests'); testQuaternionMath"
octave --quiet --eval "cd('matlab/tests'); testReactionWheelDynamics"
octave --quiet --eval "cd('matlab/simulations'); runTorqueFreeRotation"
octave --quiet --eval "cd('matlab/simulations'); runThreeWheelTest"
```

Expected quaternion-test conclusion:

```text
ALL QUATERNION TESTS PASSED
```

---

## 14. Known limitations and modeling caveats

These are not all “bugs.” Some are deliberate early-model simplifications. They matter when interpreting results.

### 14.1 Confirmed implementation issues

1. **Configured wheel speed limits are not yet validated.** The working tree contains an incomplete three-wheel limiter edit, but the committed model permits unlimited wheel speed.
2. **Relative-path execution assumption.** Simulation and older test scripts generally fail to find folders when launched from a different current directory.
3. **Unused variables/paths.** `wheelSpeed` is read but unused in `attitudeDynamicsRW`; some simulations add the tests directory but never call it.
4. **Autosave and empty artifacts.** `spacecraftParams.asv` duplicates the source, and `math/.m` is empty. Neither contributes behavior.

### 14.2 Physics not yet represented

- external disturbance torques: gravity-gradient, aerodynamic drag, solar radiation pressure, residual magnetic dipole;
- orbit and environment models;
- sensor models: gyroscope, magnetometer, Sun sensor, star tracker, GPS;
- bias, noise, scale-factor error, misalignment, latency, sampling, quantization, and dropout;
- attitude estimation or sensor fusion;
- reference attitude generation and pointing modes;
- feedback controllers;
- reaction-wheel friction, imbalance, motor electrical dynamics, torque ripple, thermal limits, and speed-dependent torque;
- momentum dumping with magnetorquers or thrusters;
- flexible structures, fuel slosh, appendages, and center-of-mass motion;
- actuator/sensor mounting errors;
- component-level mass properties or uncertainty;
- command timing, discrete flight software, or processor-in-the-loop behavior.

### 14.3 Numerical and validation limits

- Default `ode45` tolerances are used and no convergence study is present.
- Quaternion normalization is post-processing in two scripts, not constraint enforcement throughout integration.
- The torque-free initial condition is an analytically easy principal-axis case.
- Simulation invariant checks are plots or printed values, not regression assertions.
- The three-wheel script does not report total inertial momentum or quaternion norm.
- No independent implementation or experimental dataset is used for cross-validation.

### 14.4 Interpretation boundary

The model is currently appropriate for learning, equation verification, and early control-development scaffolding. It is not yet sufficient for flight qualification, actuator sizing, pointing-error budgets, hardware procurement, or claims about real on-orbit performance.

---

## 15. A disciplined path from this model to a fuller ADCS twin

The next work should preserve the existing convention and add verification at each layer.

1. **Automate dynamics invariants.** Add small tests for analytic principal-axis motion and total body-plus-wheel momentum.
2. **Enforce and validate wheel-speed limits.** Define behavior at saturation before building a controller that assumes unlimited momentum storage.
3. **Add external torque as a function of state and time.** Start with one model, such as gravity-gradient torque, and verify a limiting case.
4. **Add orbit/environment truth.** Attitude sensors require physically meaningful inertial reference vectors.
5. **Add sensor truth-to-measurement models.** Keep noise/bias parameters traceable to actual hardware candidates or requirements.
6. **Add estimation.** First validate perfect-measurement reconstruction, then introduce errors.
7. **Add attitude error and a basic controller.** Verify sign and frame convention with single-axis tests before three-axis maneuvers.
8. **Add actuator allocation and momentum management.** Generalize \(\mathbf A\) only when a real wheel layout requires it.
9. **Calibrate and validate.** Replace nominal mass properties and actuator values with measurements, uncertainty bounds, and hardware data.

Every stage should answer three questions: What equation is being implemented? What assumptions make it valid? What check would fail if the sign, frame, unit, or coupling were wrong?

---

## 16. Research reproducibility checklist

Before using a trajectory in a report or design decision, record:

- repository commit;
- MATLAB or Octave version;
- solver and explicit tolerance settings;
- initial state and its frame/units;
- spacecraft inertia tensor and provenance;
- wheel inertia, axes, limits, and provenance;
- all applied torques and saturation behavior;
- quaternion convention and transform direction;
- conserved-quantity errors;
- sensitivity to smaller tolerances/step sizes;
- known omitted physics relevant to the conclusion.

For attitude plots, remember that \(\mathbf q\) and \(-\mathbf q\) are the same orientation. A component sign flip is not necessarily a physical discontinuity. For error metrics, use a quaternion or principal rotation angle that respects this double cover.

---

## 17. External references

The code is self-contained; these sources provide the broader theory and tool context.

1. Malcolm D. Shuster, “A Survey of Attitude Representations,” *The Journal of the Astronautical Sciences*, 41(4), 1993, pp. 439–517. A foundational comparison of spacecraft attitude representations.
2. F. Landis Markley and John L. Crassidis, *Fundamentals of Spacecraft Attitude Determination and Control*, Springer, 2014, [DOI: 10.1007/978-1-4939-0802-8](https://doi.org/10.1007/978-1-4939-0802-8).
3. James R. Wertz (ed.), *Spacecraft Attitude Determination and Control*, Springer/Kluwer, 1978, [DOI: 10.1007/978-94-009-9907-7](https://doi.org/10.1007/978-94-009-9907-7).
4. NASA Small Spacecraft Systems Virtual Institute, [Guidance, Navigation, and Control: State of the Art of Small Spacecraft Technology](https://www.nasa.gov/smallsat-institute/sst-soa/guidance-navigation-and-control/). Context for real small-spacecraft sensors, actuators, and ADCS architectures.
5. MathWorks, [`ode45` documentation](https://www.mathworks.com/help/matlab/ref/ode45.html). Solver interface, algorithms, options, and limitations.
6. The CubeSat Program, Cal Poly San Luis Obispo, [CubeSat Design Specification program page](https://www.cubesat.org/). Mechanical standard context for CubeSat form factors; compliance is not established by this repository.

---

## 18. Final mental model

The repository can be understood as four stacked layers:

```text
CONVENTIONS
  define what every coordinate and quaternion means
        ↓
MATH UTILITIES
  compose, invert, normalize, and convert attitude
        ↓
DYNAMICS
  turn torques and momentum into q-dot and omega-dot
        ↓
SIMULATIONS + TESTS
  integrate trajectories and check known mathematical properties
```

At the bare-spacecraft level, the central equations are

\[
\dot{\mathbf q}=\frac12\mathbf q\otimes[0,\boldsymbol\omega]^T,
\qquad
\mathbf J\dot{\boldsymbol\omega}
+\boldsymbol\omega\times(\mathbf J\boldsymbol\omega)=\boldsymbol\tau.
\]

With reaction wheels, internal momentum becomes

\[
\mathbf H_{\text{total}}^B
=\mathbf J\boldsymbol\omega
+\mathbf A\mathbf J_w\boldsymbol\Omega,
\]

and motor torque moves momentum between wheels and spacecraft without creating net inertial angular momentum.

That is what has been built so far: a coherent quaternion convention, tested rotation utilities, a nonlinear rigid-body plant, ideal reaction-wheel momentum exchange, and three demonstration scripts. The strongest next improvement is turning the remaining physical invariants and actuator limits into automated dynamics tests.
