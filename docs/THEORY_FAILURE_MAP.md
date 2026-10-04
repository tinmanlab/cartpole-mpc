# Theory failure map

The purpose of this map is not to make CartPole complicated. It is to prevent a complex robot project from treating every failure as "bad gains" or "reality gap".

For each failure class, the first question is: **what observation would falsify this hypothesis?**

| Failure class | Typical symptom | Smallest diagnostic experiment | Wrong response | Robot-scale response |
|---|---|---|---|---|
| wrong model structure | no parameter set fits held-out dynamics | compare residual structure across excitation regimes | keep tuning Q/R | change model class: compliance, actuator state, contact, DAE |
| wrong physical parameters | one-step/rollout error improves strongly after identification | nominal vs identified model on held-out excitation | let controller weight compensate | physically constrained SysID |
| poor identifiability | many parameter sets fit equally well | sensitivity/Fisher-information/condition analysis | trust optimizer output | redesign excitation or parameterization |
| unobservable state | estimator drift persists regardless Q/R | observability/gauge analysis + oracle measurement arm | increase filter gain | add/alter measurement or state definition |
| estimator miscalibration | low RMSE but inconsistent P/NIS/NEES | held-out consistency check | optimize RMSE only | calibrate covariance/kinematics |
| wrong noise model | biased/correlated innovations | innovation whiteness / distribution test | assume white Gaussian | bias state, colored-noise model, robust likelihood |
| measurement outlier | one bad sample produces large correction | Gaussian-noise vs deterministic-glitch arm | globally increase R | gating, robust loss, reliability model |
| timestamp/skew | estimates good offline but phase-lagged in loop | aligned replay vs shifted timestamps | retune controller | time-offset calibration / delay-aware estimator |
| actuator lag/gain | command and applied effort differ | ideal command vs identified actuator model | raise feedback gain | identify current/torque loop and augment model |
| saturation / torque-speed | aggressive command clips or weakens at speed | low-speed vs high-speed same task | increase Q tracking | explicit input envelope / actuator constraints |
| latency/jitter | stable offline, unstable under transport load | no-delay vs measured delay/jitter | blame solver | delay compensation / prediction / scheduling |
| discretization error | behavior changes materially with dt/integrator | timestep refinement / integrator comparison | retune gains at one dt | validated discretization and integration error budget |
| local-linear-model failure | KF/LQR works near equilibrium only | small vs large excursion | call linear and nonlinear methods equivalent | LTV/SQP-RTI or nonlinear OCP |
| controller-objective error | desired variable good, another state drifts | add explicit state metric | crank arbitrary weight | reformulate engineering objective |
| missing constraint | nominal stable, recovery violates physical boundary | cost-only vs explicit constraint arm | huge penalty only | hard/soft state/path constraints + feasibility |
| infeasibility | solver suddenly returns poor command | log primal/dual residual and constraint margins | treat as timeout | fallback/recovery policy and feasibility restoration |
| local optimum / globalization | different warm starts yield different NMPC solution | multi-start / line-search diagnostic | trust one solution | globalization, trust region, multiple shooting |
| numerical scaling | solver iterations/residuals explode | nondimensionalized vs raw units | change control weights | scale states/constraints and inspect KKT residuals |
| deadline miss | mathematically valid command arrives late | solver-only vs real scheduler load | benchmark average time | p50/p95/p99 + deadline admission |
| hybrid/contact mismatch | stance update helps sometimes, corrupts sometimes | scheduled contact vs sensed reliability | one binary contact flag | separate mode hypothesis, measurement, reliability |
| slip/partial contact | FK constraint becomes biased | clean stance vs slip arm | binary reject only | directional covariance / robust contact estimation |
| impact/reset mismatch | transient divergence at touchdown | pre/post-impact replay | inflate all Q/R | jump/reset dynamics or impact-aware estimator |
| environment/friction mismatch | tracking fails on terrain change | identified friction vs held-out surface | randomize arbitrarily | measured uncertainty + robust/scenario treatment |
| domain-randomization overfit | randomized training passes, real replay fails | train distribution vs wider held-out OOD | randomize even wider blindly | identify missing factor first |
| tuning overfit | train improves, validation/test worsens | disjoint seed/scenario split | deploy best train score | reject candidate |
| context dependence | one gain set works for one gait only | evaluate same gains over contexts | one universal optimum | gain scheduling/contextual tuning |
| estimator-controller coupling | standalone estimator/controller pass, combined loop fails | oracle state vs estimator-in-loop | tune blocks independently forever | closed-loop co-tuning after staged validation |
| runtime fault / sensor failure | abrupt residual regime change | injected dropout/stuck/bias fault | absorb into covariance forever | fault detection/isolation + degraded mode |
| thermal/power derating | long-duration performance decays | short vs sustained run | retune cold system | thermal/current/power model and derating |
| structural safety gap | nominal controller works but unsafe state reachable | disturbance boundary / reachability check | average-cost tuning | safety filter / backup / reachability where required |

## Five model questions before changing any controller

1. Is the **model class** capable of representing the observed behavior?
2. Are its important parameters **identifiable with the data we collected**?
3. Are the controller/estimator using the **same signal timing and actuator contract** as the plant?
4. Are failures caused by **constraints or mode changes** rather than the nominal dynamics?
5. Does the candidate still pass on **held-out conditions that were not used to select it**?

If any answer is unknown, gain tuning is premature.

## Linear vs nonlinear is not binary

A transferable control stack distinguishes:

    fixed LTI
        one A,B around one operating point

    LTV / successive linearization
        A_k,B_k recomputed along a nominal trajectory

    nonlinear OCP
        x_(k+1)=f(x_k,u_k) remains in the optimization;
        local derivatives are still used by SQP/DDP/iLQR

    hybrid / switched nonlinear OCP
        nonlinear dynamics + discrete modes + jump/contact constraints

For a humanoid, "we use NMPC" is incomplete unless it also states:
- which state/input model,
- which contact/mode representation,
- which constraints,
- which discretization,
- which solver/globalization,
- which estimator signal contract,
- which real-time deadline.

## Estimator family choice is also structural

    KF
      linear Gaussian

    EKF / ESKF
      nonlinear prediction with local error-coordinate Jacobians; error geometry is a separate choice

    InEKF
      invariant error when Lie-group/group-affine structure is useful

    UKF / sigma-point
      multiple-point nonlinear propagation without explicit Jacobians; measured cost depends on the model

    MHE / NMHE
      recent-window optimization, useful for explicit constraints and parameter/state coupling

    factor graph / smoothing
      asynchronous long-window multi-sensor inference

    particle / mixture
      non-Gaussian or multimodal belief when Gaussian filters are structurally wrong

A more complicated estimator is not automatically better. Use the simplest family that represents the uncertainty and geometry actually present.

## Robustness methods answer different uncertainty questions

- **Tube MPC**: bounded additive disturbance around a nominal trajectory.
- **Min-max MPC**: optimize against a worst-case uncertainty/adversary.
- **Scenario MPC**: enforce/evaluate sampled uncertainty realizations.
- **Chance-constrained MPC**: constrain violation probability under a probabilistic uncertainty model.
- **Risk-sensitive/CVaR**: weight tail outcomes in the objective.
- **Adaptive MPC**: update uncertain model parameters online.
- **Robust estimation**: change the estimator likelihood/loss rather than the controller.

Do not call domain randomization "robust MPC". It is an experiment/training distribution unless the control law itself has a robustness formulation.

## The deployment order

    structural model sanity
    → physical + actuator + timing identification
    → sensor/kinematic calibration
    → observability / estimator consistency
    → oracle-state controller admission
    → explicit constraints / feasibility
    → estimator-in-loop admission
    → one realism factor at a time
    → combined held-out stress
    → scheduler/deadline stress
    → hardware replay / shadow mode
    → low-energy safe trials
    → safe residual tuning
    → runtime adaptation and fault handling

This is an iterative diagnostic template, not a universally required ordering or algorithm list.

## Limits of diagnosis labels

An observed symptom need not identify a unique cause. Successful local rank/fit tests do not establish nonlinear global observability, identifiability or recovery. Numerical rejection, solver-reported infeasibility and a true plant-envelope violation are separate observations; a nonlinear solver failure does not prove global infeasibility. The local soft band is goal-relative while hard rail is world-relative. Improving one metric does not justify changing this distinction or declaring safety.
