# Figure transfer map

Purpose: use CartPole as a small commissioning laboratory, then replace each educational implementation with the native authority appropriate for a floating-base multi-contact robot.

This document does **not** assert that the local Figure checkout is current upstream state. The mapping below is anchored to the locally verified Figure evidence dated 2026-09-29 through 2026-10-01.

## Transfer principle

Do not copy CartPole equations into Figure.

Transfer the contract:

    model
    → identified parameters + uncertainty
    → estimator
    → controller
    → actuator / transport
    → plant
    → independent scoring

and replace each implementation with a robot-scale native implementation.

| CartPole concept | Figure / humanoid equivalent | Native authority |
|---|---|---|
| nonlinear plant | floating-base rigid-body + contact plant | MuJoCo / robot descriptor |
| fixed LTI model | one equilibrium/contact-mode linearization | analytical/AD rigid-body derivatives |
| LTV / RTI bridge | relinearize along current trajectory/contact mode | OCS2/acados SQP-RTI |
| Full NMPC | full-order floating-base OCP | OCS2 full-order / acados + Pinocchio |
| centroidal-style MPC | centroidal momentum/contact-wrench OCP | OCS2 centroidal stack |
| soft cart bound | joint/contact/collision/path constraints | native OCP constraints |
| robust-MPC concept | model/contact/estimator uncertainty margins | robust/scenario/chance formulation as justified |
| KF/EKF | stock-sensor floating-base estimator | ESKF/EKF implementation |
| UKF | sigma-point nonlinear Gaussian alternative | use only if Gaussian belief remains appropriate and Jacobian-free propagation adds value |
| SO(2) bridge | SO(3)/SE(3)/SE_2(3) invariant error | InEKF when structure applies |
| shooting-MHE bridge | constrained windowed velocity/contact estimation | OSQP/acados MHE or equivalent |
| Adaptive-R bridge | contact/FK reliability covariance | residual adaptation / FOCUS / CoCo where evidence supports it |
| system ID | link inertia, kinematic, actuator and latency ID | physically constrained SysID + actuator repo |
| CEM calibration | covariance/kinematics calibration | bilevel / likelihood / offline calibration |
| black-box gain tuning | residual hardware feedback gain tuning | Safe BO after a safe seed exists |
| sim2real scenario | real-signal, actuator, timing, contact and model mismatch brackets | Figure evidence harness |
| advanced failure injection | colored noise, stale/stuck sensors, jitter, torque-speed, thermal derating, discretization | measured telemetry + actuator/timing/native sensor models |

## Current Figure evidence this should address

### State-estimation boundary

The 2026-10-01 local Figure evidence showed:
- truth-base realistic arm: 12/12 at stand, walk 0.3 and 0.5,
- estimator + planned contact: 12/12 stand, 12/12 at 0.3, 0/12 at 0.5,
- estimator + torque contact: 8/12 stand, 2/12 at 0.3, 0/12 at 0.5.

This means controller development cannot be declared complete from oracle-state success.

CartPole transfer:
- always keep Truth/oracle as a diagnostic arm only,
- tune/calibrate estimator separately,
- run controller tuning with estimator in the loop,
- preserve NIS/NEES or equivalent consistency diagnostics,
- separate contact/reliability failure from generic process noise.

### Contact/reliability boundary

The same Figure evidence reported planned contact agreeing with plant truth much more often than the simple torque heuristic.

CartPole cannot reproduce multi-contact physics. The reusable lesson is to keep these interfaces separate:

    contact/mode hypothesis
    measurement construction
    measurement reliability/covariance
    estimator update

Do not collapse all four into one "contact detector" flag.

For Figure:
- planned contact is a baseline/schedule source,
- torque/current-derived contact is a separate measured source,
- FK/velocity reliability should be represented continuously when justified,
- plant contact truth remains scorer-only.

### Actuator boundary

Figure evidence still listed missing or incomplete:
- torque-speed behavior,
- friction fitted to platform data,
- command latency/jitter,
- current-loop effects,
- thermal behavior.

CartPole's sim2real stack provides slots for actuator gain, lag, command delay, friction and command-vs-applied diagnostics. Figure should replace these with G1-appropriate data and the actuator model rather than reuse the CartPole numbers.

### Real-time boundary

The OCS2 centroidal evidence showed a roughly 12.5 ms MPC slot with solve times close enough to the deadline that scheduling and concurrency changed behavior.

Therefore commissioning acceptance needs both:

    task/control metrics
    AND
    timing/deadline metrics

A controller that is dynamically valid but misses its real-time contract is not admitted.

### Constraint boundary

CartPole commissioning found that input-bounded MPC can stabilize the pole while allowing the cart to drift to the track boundary.

The humanoid analogue is more serious:
- torque/velocity limits,
- friction cone,
- unilateral contact,
- stance/swing constraints,
- self/environment collision,
- support/CoP constraints,
- thermal/power limits where relevant.

Do not replace a safety constraint with a very large cost weight merely to make the nominal simulation pass.

## Figure commissioning order

A reusable order for the model-based lane is:

1. **Descriptor/kinematic sanity**
   - frame conventions,
   - joint order,
   - limits,
   - mass/inertia consistency.

2. **Actuator and timing identification**
   - identify command → applied effort separately from rigid-body parameters,
   - effective low-level gains/bandwidth,
   - torque-speed envelope,
   - armature/reflected inertia,
   - friction/compliance/backlash where material,
   - command/state delay and jitter,
   - validate on a held-out excitation trajectory.

3. **Estimator calibration**
   - stock signal contract only,
   - Q/R and bias process calibration,
   - contact/reliability covariance,
   - kinematic offsets,
   - held-out trajectory consistency.

4. **Controller nominal admission**
   - oracle state first for algorithm isolation,
   - explicit path constraints,
   - nominal timing/deadline.

5. **Estimator-in-loop admission**
   - same controller and plant,
   - no simulator-only state leakage.

6. **Single-factor realism ablations**
   - sensor noise/bias,
   - contact timing/slip,
   - actuator dynamics,
   - latency/jitter,
   - model/inertia mismatch,
   - floor/friction/terrain.

7. **Combined held-out stress**
   - unseen seeds,
   - uncertainty distribution broader than tuning distribution,
   - report worst/quantile behavior, not only mean.

8. **Hardware-safe residual tuning**
   - only after a safe starting set exists,
   - constrained/Safe BO for gains or other black-box parameters,
   - no uncontrolled search over structural parameters.

9. **Runtime adaptation**
   - covariance/reliability,
   - bias,
   - temperature/derating,
   - context-dependent gains only when required.

## What CartPole should never become

Do not turn this repository into:
- a second OCS2,
- a second Pinocchio,
- a humanoid contact simulator,
- a hardware safety framework,
- a second source of Figure-specific parameters.

Its value is a small executable reference for the **reasoning structure and acceptance contract**.

## Additional commissioning transfers

- UKF is available as a sigma-point nonlinear Gaussian alternative, but should only be admitted when Jacobian/local-linearization error is the relevant limitation; hybrid contact ambiguity is not solved by UKF.
- Colored noise, stale/dropout packets and stuck sensors require explicit timestamp/fault evidence, not only larger R.
- Torque-speed/current/voltage and thermal derating belong to the actuator/input-authority model used by control and acceptance tests.
- Integration/timestep refinement is a prerequisite before interpreting a controller retune as a physics improvement.
- The CartPole NMPC + backup supervisor transfers as an architectural pattern only: Figure needs native constraint/safety semantics and verified fallback behavior.
- Joint controller-estimator tuning is a late-stage optimization. Keep staged model/estimator/controller evidence so a joint optimizer cannot hide which block is wrong.
