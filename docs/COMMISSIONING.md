# Commissioning: from a correct algorithm to a transferable robot stack

This repository separates algorithm correctness from commissioning.

A controller or estimator can be mathematically correct and still fail on hardware because the model is wrong, covariance is miscalibrated, constraints are missing, latency or actuator dynamics are ignored, or parameters were tuned on one condition and do not generalize.

The commissioning path is:

    model structure
      ↓
    parameter identification
      ↓
    estimator calibration
      ↓
    controller tuning
      ↓
    estimator-in-the-loop closed-loop validation
      ↓
    held-out sim2real stress
      ↓
    safe hardware residual tuning
      ↓
    runtime adaptation

## 1. Model identification comes before controller tuning

A weight such as MPC Q can accidentally compensate for a wrong mass, actuator gain, or latency. That can produce a controller that looks good in one simulator but encodes the wrong physics.

The offline commissioning harness therefore starts with plant identification.

src/commissioning.js generates an excited CartPole trajectory and estimates:

    mc        cart mass
    mp        pole mass
    l         pole COM length
    friction  viscous cart friction

using constrained derivative-free optimization against one-step transition error.

The receipt also runs a separate command-to-applied-actuation identification experiment. It estimates:
- actuator gain,
- first-order actuator response coefficient,
- discrete command delay.

This separation is deliberate: a controller cost weight must not silently compensate for actuator bandwidth or transport delay.

A humanoid version should instead use physically constrained identification of link inertias and center of mass, kinematic offsets, actuator constants, reflected inertia, friction/compliance/backlash, command/state latency, and power/thermal effects when material.

Randomization does not replace system identification. The identified model and residual uncertainty define the center and width of the randomization distribution.

## 2. Estimator calibration is not "make RMSE small"

Estimator parameters are written separately from controller cost weights:

    Q_e  process-noise covariance
    R_e  measurement-noise covariance
    P0   initial state covariance

This avoids confusing them with controller Q_c and R_c.

The commissioning harness tunes Q_e and an R_e scale using closed-loop state-estimation error. It also penalizes inconsistency using:

    NIS = innovation^T S^-1 innovation

and

    NEES = error^T P^-1 error.

For an ideal Gaussian filter these statistics should be compatible with their corresponding chi-squared dimensions. The CartPole objective uses a compact deviation penalty rather than pretending that one finite simulation is a formal statistical test.

A candidate is accepted only if it improves held-out validation and held-out test.

### Humanoid path

For legged/humanoid robots, stronger methods should replace the educational CEM tuner:

- covariance/kinematic bilevel calibration:
  https://arxiv.org/abs/2510.11539
- classical covariance identification / EM when assumptions fit,
- online innovation/residual adaptation:
  https://arxiv.org/abs/2608.02316
- learned contact/reliability covariance when justified by data.

The 2026 legged bilevel-calibration work jointly optimizes covariance and kinematic parameters through an estimator-in-the-loop objective. That is closer to the intended Figure path than hand-tuning Q_e/R_e.

## 3. Controller tuning must use closed-loop objectives

An MPC solves an open-loop horizon cost at every control tick, but the engineering objective is usually a long-horizon closed-loop metric.

The CartPole controller score combines tracking, maximum position excursion, control effort, estimator error, failure, and command-to-applied-command mismatch.

It is risk-sensitive: the mean and worst observed condition both matter.

The educational CEM tuner adjusts LQR cost parameters to demonstrate continuous closed-loop tuning.

A second executable tuner performs a small structured search for Linear MPC over:
- horizon N,
- input-effort weight scale,
- angle-state cost scale.

This deliberately separates smooth weight tuning from structural/discrete choices and applies the same held-out admission rule.

For a larger robot, use a method appropriate to the parameter type:

| Parameter family | Recommended method |
|---|---|
| smooth MPC cost weights | differentiable closed-loop tuning |
| horizon / mode / solver choices | structured search / BO |
| hardware feedback gains | safe Bayesian optimization |
| contact/gait-dependent gains | contextual safe BO / gain scheduling |
| model parameters | system identification |
| estimator covariance/kinematics | likelihood or bilevel calibration |

Relevant references:
- DiffTune-MPC: https://arxiv.org/abs/2312.11384
- Safe Bayesian tuning on legged hardware: https://arxiv.org/abs/2306.07092
- control-estimation co-design: https://arxiv.org/abs/2609.36090

## 4. Train, validation, and test remain separate

The commissioning script uses disjoint seeds and conditions.

The acceptance rule is:

    tuned validation < baseline validation
    AND
    tuned test < baseline test

If the optimizer improves only its training conditions, the candidate is rejected.

This prevents a controller from hiding model error by overfitting one disturbance distribution.

## 5. Sim2real failure factors are ablated before being combined

The live lab now exposes the following **single-factor** diagnostic scenarios before the combined stack:
- model mismatch (mass / pole-mass / length / friction),
- sensor white noise,
- sensor bias random walk,
- actuator lag + gain error,
- two-tick command latency,
- deterministic measurement glitch.

The combined sim2real scenario then combines hidden:
- mass / pole-mass / length mismatch,
- friction,
- command delay,
- first-order actuator lag,
- actuator gain error,
- sensor noise,
- sensor bias random walk.

External push is added by the experiment when desired.

The commissioning receipt includes a paired single-factor ablation for LQR+EKF and Full-NMPC+EKF. Diagnose with those rows first; use the combined stack only as a final interaction test.

Record both commanded u and applied u because a controller cannot compensate correctly for dynamics that never enter its model or evidence.

For humanoids, extend the same slots with torque-speed envelope, motor current-loop bandwidth, reflected inertia, thermal derating, transmission elasticity/backlash, encoder offset, IMU bias/vibration, contact/slip uncertainty, control/comms jitter, floor friction/compliance, link inertial uncertainty, and perception delay/dropouts.

## 6. Do not jump directly from simulation tuning to hardware

A practical deployment ladder is:

    oracle state + nominal actuator
    → sensor model
    → estimator in loop
    → actuator dynamics
    → latency / jitter
    → model uncertainty
    → contact uncertainty
    → held-out randomization
    → shadow hardware replay
    → low-energy constrained hardware tests
    → safe residual tuning
    → wider task envelope

Each layer retains the previous evidence so a regression can be localized.

## 7. Joint control-estimation co-design is a later stage

Sequential calibration is easier to debug:
1. model,
2. estimator,
3. controller,
4. closed-loop interaction.

After these pieces are individually trustworthy, joint design can improve the result.

Recent work such as ContEst formulates actuation, sensing, estimation, and control in a coupled optimization:
https://arxiv.org/abs/2609.36090

Its nonlinear extension still relies on EKF/MPC approximations and local linearization, so this repo treats joint co-design as an advanced layer rather than a replacement for the debugging ladder above.

## Run

    node scripts/run_commissioning.js

The receipt is written to:

    evidence/commissioning.json

It records plant and actuator identification, identifiability diagnostics, estimator calibration candidate and held-out gate, controller tuning candidate and held-out gate, single-factor ablations, timing tails, and controller × observer × stress-scenario results.

The parameters in this evidence file are CartPole-specific and must not be copied into Figure.

## Current commissioning receipt

The current deterministic receipt intentionally demonstrates both successful and rejected tuning.

System identification:
- true hidden model: mc=1.25, mp=0.08, l=0.575, friction=0.15,
- identified values are close to the hidden model,
- held-out one-step loss drops by several orders of magnitude relative to the nominal model,
- a residual-sensitivity correlation matrix is recorded so apparent optimizer convergence is not confused with parameter identifiability.

Estimator calibration:
- accepted by the held-out gate,
- validation/test commissioning score improves relative to the baseline,
- the objective includes NIS/NEES consistency rather than RMSE alone,
- the receipt also reports the fraction of NIS/NEES samples inside the nominal 95% chi-square interval as a diagnostic (not as an independence assumption or formal proof).

Controller black-box tuning:
- training score improves,
- validation and test scores get worse,
- candidate is therefore rejected.

This rejection is a feature of the commissioning contract, not a tuner failure to hide.

### Stress boundary found

In the fixed three-seed combined sim2real stress:
- Linear MPC reaches the track/failure boundary in 2/3 runs,
- LTV MPC reaches it in 1/3,
- Full NMPC reaches it in 0/3,
- LQR reaches it in 0/3.

This is not an algorithm ranking. Inspection shows that the predictive controllers can prioritize pole recovery while allowing large cart drift. The current formulations have input bounds but no explicit hard cart-position path constraint.

Therefore the next controller admission gate is:

    explicit state/path constraints
    → soft vs hard constraint behavior
    → feasibility / terminal treatment
    → robust/chance-constrained variants when uncertainty matters

For humanoids this maps to joint limits, torque-speed limits, contact/friction constraints, collision avoidance, support constraints, and solver feasibility.

## Robustification admission

The commissioning lane also compares the nominal Linear MPC with the executable finite-model Scenario-risk MPC on paired model-mismatch and combined sim2real seeds. The candidate is accepted only if failure count and maximum cart excursion do not worsen in either held-out condition. This is intentionally strict: adding uncertain models to an objective is not a robustness guarantee.
## Advanced failure injection

The commissioning harness now separates several failures that are often incorrectly collapsed into generic noise or reality gap:

- AR(1) colored sensor noise, with innovation lag-1 correlation recorded,
- packet dropout represented as hold-last plus stale/freshness metadata,
- stuck sensor values, distinct from packet loss,
- stochastic command-hold jitter,
- state-dependent torque/force authority through a torque-speed envelope,
- history-dependent thermal derating,
- integration/discretization refinement from 1 to 16 plant substeps.

These are simplified CartPole mechanisms. The transferable contract is the diagnostic separation: a sensor model, transport model, actuator envelope, thermal state and numerical integration error should not be hidden inside controller gain tuning.

The commissioning harness also includes a low-dimensional controller-estimator co-tuning bridge. It is accepted only if the joint candidate improves both held-out validation and test. This is intentionally not called ContEst or DiffTune; those methods should replace the small black-box bridge when their assumptions and native solver sensitivities are appropriate.
