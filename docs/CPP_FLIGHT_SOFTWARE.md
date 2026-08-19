# Portable C++ ADCS Flight Core

## Scope

The C++17 core executes the non-environment ADCS algorithms without MATLAB,
Simulink, Python, an operating system, or hidden global state. Eigen supplies
fixed-size linear algebra. GoogleTest is linked only into the Linux SIL test
executable and is not part of flight code.

All vectors use SI units and `q_IB=[qw,qx,qy,qz]` is scalar-first Hamilton,
rotating body coordinates into ECI. Headers are under `cpp/include/adcs`:

| Header | Deterministic interface |
|---|---|
| `math.hpp` | quaternion algebra, DCM conversion, skew matrix |
| `estimation.hpp` | TRIAD/QUEST reference solution and six-state gyro-bias MEKF |
| `guidance.hpp` | inertial, Sun/safe, nadir/LVLH, cubic-time slew references |
| `control.hpp` | quaternion PD, configured LQR, fixed-size wheel allocation |
| `mode.hpp` | initialization/detumble/safe/nominal/slew/desaturation/fault logic |
| `config.hpp` | explicit gains, limits, noise, and mode thresholds |
| `flight.hpp` | measurement, reference, actuator, estimator, and mode boundary |
| `protocol.hpp`, `timing.hpp` | versioned wire structs and rollover-safe timing |

`FlightComputer` owns only its configuration, MEKF state, and mode state.
Callers timestamp/schedule inputs, call `predict_gyro` at the gyro rate, call
`update_vector` for valid asynchronous observations, update mode status, and
request bounded wheel commands from an estimated body rate and attitude
reference. Truth state is not an input type.

The LQR gain is computed and checked in MATLAB from the documented CARE model,
then stored explicitly in `FlightConfig`; flight code does not solve a Riccati
equation at runtime. The selected three-wheel allocator and MEKF use fixed-size
matrices. QUEST accepts a variable observation count as an initialization/
reference utility and is not in the periodic `FlightComputer` path.

## Build and tests

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

The build enables `-Wall -Wextra -Wpedantic -Werror`. `adcs_sil` is a
GoogleTest executable covering math/frames, QUEST/MEKF, guidance, PD/LQR,
allocation, modes, flight interfaces, protocol/timing, and MATLAB parity.

## MATLAB cross-validation

`generateCppReferenceVectors.m` writes deterministic results to
`cpp/tests/data/matlab_reference.csv`. The C++ cross-validation tests recompute
the same cases independently and enforce:

| Quantity | Absolute tolerance |
|---|---:|
| Quaternion/DCM/basic control | `1e-14` to `2e-15` |
| Guidance quaternions/rates | `1e-12` to `1e-13` |
| MEKF quaternion/bias/covariance | `1e-13` to `2e-13` |
| MEKF NIS | `2e-10` |
| Allocation | `1e-15 N m` |
| Mode sequence | exact enumeration match |

The CSV is committed so C++ tests have no runtime MATLAB dependency. Regenerate
it only when the MATLAB reference intentionally changes, review the diff, then
rerun both MATLAB and C++ suites.
