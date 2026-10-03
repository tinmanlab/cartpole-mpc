# Tuning methods: choose the optimizer to match the parameter

There is no single "SOTA hyperparameter tuner" for a robot stack.

The correct question is:

> What does this parameter mean physically, is the closed-loop map differentiable, is the variable continuous or discrete, and can unsafe trials occur?

## Separate controller and estimator notation

Controller cost:

    Q_c, R_c, Q_f

Estimator noise:

    Q_e, R_e, P0

Using Q and R for both without a subscript is convenient in equations but dangerous in system commissioning.

## Parameter families

| Parameter | Preferred route | Why |
|---|---|---|
| smooth MPC cost weights | differentiable closed-loop tuning | gradients are informative and sample efficient |
| horizon / solver / mode choices | structured search / BO | discrete choices |
| soft constraint weights | differentiable or constrained search | continuous but coupled to feasibility |
| hardware PD/residual gains | Safe Bayesian optimization | black-box and safety-critical |
| context-dependent gains | contextual BO / scheduling | optimum changes with gait/task |
| Q_e / R_e / kinematic offsets | likelihood or bilevel calibration | estimator-in-the-loop problem |
| time-varying covariance | innovation/residual or learned reliability | online nonstationarity |
| mass / inertia / friction / latency | system identification | physical parameter, not "controller tuning" |

## 1. Closed-loop differentiable MPC tuning

DiffTune-MPC optimizes MPC cost parameters against a longer-horizon closed-loop performance objective, rather than equating the MPC's short open-loop cost with engineering performance.

Reference:
- DiffTune-MPC: https://arxiv.org/abs/2312.11384

Relevant idea:

    theta <- projection(theta - alpha * d L_closed_loop / d theta)

where theta can parameterize Q_c/R_c and other smooth cost terms.

Its nonlinear-MPC extension differentiates the MPC policy through local/SQP structure.

Use this when:
- the model and solver sensitivities are trustworthy,
- parameters are continuous,
- you can evaluate a differentiable closed-loop objective.

Do not use it blindly for:
- solver choice,
- contact-mode combinatorics,
- hardware exploration with unsafe candidates.

## 2. Safe Bayesian optimization for hardware residual gains

For hardware gains, the objective and constraints may be unknown black boxes.

A legged-robot example uses contextual GOSAFEOPT to tune feedback gains around an MPC + WBC controller, using gait parameters as context.

Reference:
- Tuning Legged Locomotion Controllers via Safe Bayesian Optimization:
  https://arxiv.org/abs/2306.07092

The paper reports hardware tuning for trot and crawl while preserving the method's safe-exploration contract, and demonstrates that different gait contexts prefer different gains.

Use this after:
- a known safe seed exists,
- structural model/controller errors are already resolved,
- safety metrics are measurable online.

A theoretical Safe-BO guarantee still depends on assumptions; governance and conservative hardware limits remain necessary.

## 3. Estimator covariance and kinematic calibration

For a legged estimator, Q_e/R_e and kinematic offsets should not default to perpetual hand tuning.

A 2026 bilevel method places covariance and kinematics in an outer calibration problem and a MAP/full-information estimator in the inner problem.

Reference:
- Simultaneous Calibration of Noise Covariance and Kinematics for State Estimation of Legged Robots via Bi-level Optimization:
  https://arxiv.org/abs/2510.11539

The outer objective compares the estimated trajectory against ground truth while maintaining positive-definite covariance constraints.

This is a much closer model for Figure commissioning than copying CartPole Q_e/R_e numbers.

## 4. Online adaptive covariance

Offline calibration produces a baseline covariance model. Leg contact quality and impacts are time varying.

Reference:
- Residual-Based Adaptive Kalman Filtering for Legged Robot State Estimation:
  https://arxiv.org/abs/2608.02316

The method adapts Q_e/R_e from innovation and residual statistics in an InEKF and reports real quadruped experiments. In that study, R_e adaptation was the most useful of the tested variants.

This should be interpreted as runtime adaptation around a calibrated estimator, not proof that offline calibration is unnecessary.

## 5. Control-estimation co-design

Sequential commissioning is easier to diagnose:

    model -> estimator -> controller -> interaction

After these blocks are trustworthy, joint optimization can expose coupled optima.

Reference:
- Control and Estimation Co-Design via Envelope-Theorem Gradients:
  https://arxiv.org/abs/2609.36090

ContEst jointly treats actuation, sensing and estimation in a two-stage design objective. Its nonlinear constrained extension still uses EKF/MPC approximations and local linearization, and the paper itself discusses scaling limits.

For a high-dimensional humanoid this is currently a research direction, not a replacement for staged commissioning.

## 6. The tuner objective is not one RMSE

A controller objective should cover task performance and engineering cost, for example:

    J =
      w_track * tracking
    + w_effort * effort
    + w_smooth * command_rate
    + w_recovery * disturbance_recovery
    + w_model * command_application_mismatch
    + failure_penalty

Safety requirements should preferably be constraints, not only huge weights:

    torque <= limit
    velocity <= limit
    contact force in feasible set
    collision margin >= minimum
    solver deadline <= period

Estimator calibration should include:
- state error,
- innovation statistics,
- NIS,
- NEES,
- bias/drift,
- held-out consistency.

## 7. Train / validation / test is part of control engineering

The current CartPole commissioning lane uses different seeds and conditions for:
- tuner training,
- validation,
- final held-out test.

Candidate acceptance requires held-out improvement.

This is not an ML-only convention. It is necessary whenever parameters are selected from simulation/data.

## 8. Uncertainty distribution must be justified

Do not choose a randomization interval because it makes the controller robust.

Recommended order:

    measured/identified central value
    → parameter confidence / repeatability
    → conservative uncertainty bracket
    → training/randomization distribution
    → broader held-out stress distribution

For real robots, uncertainty intervals should come from:
- SysID repeatability,
- calibration residuals,
- datasheet/physical bounds,
- operating-temperature or payload ranges,
- measured latency/jitter distributions.

## 9. What this repository executes

The browser lab executes the controller/estimator algorithms.

The slower commissioning harness executes:
- constrained derivative-free CartPole system ID,
- Q_e/R_e calibration with RMSE + NIS/NEES penalty,
- closed-loop LQR cost tuning,
- structured Linear-MPC hyperparameter search over horizon, effort scale and angle-state cost scale,
- a fixed initial-angle model-hierarchy sweep across LQR / fixed-linear MPC / state-aware MPC / LTV-RTI / full NMPC,
- held-out accept/reject gates,
- single-factor sim2real ablation,
- combined stress.

The structured MPC search is intentionally small. It demonstrates that horizon and cost choices are selected on train data and must still pass validation/test; it is not presented as a replacement for differentiable MPC tuning or solver-native BO on a humanoid.

CEM is deliberately used as a dependency-free teaching optimizer. It demonstrates the contract; it is not presented as the preferred optimizer for a humanoid.

For Figure:
- differentiable MPC tuning -> native OCS2/acados sensitivity path when justified,
- black-box hardware gains -> Safe BO,
- estimator covariance/kinematics -> bilevel/likelihood calibration,
- model parameters -> physically constrained SysID.

## 10. Robustness formulation is itself a hypothesis

The executable Scenario-risk MPC is intentionally admitted through the same held-out logic as a tuned controller. A finite uncertainty ensemble can make results worse if its model set, risk measure, constraints or compute budget are wrong. The commissioning receipt therefore records an explicit accept/reject result for this robustification candidate rather than assuming that more scenarios imply more robustness.
