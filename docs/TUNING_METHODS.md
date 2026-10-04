# Tuning methods: conditional tools, not a mandatory SOTA recipe

Separate controller costs Q_c/R_c/P_f from estimator noise Q_e/R_e/P0. Physical parameters, numerical settings, control objectives and uncertainty have distinct meanings even when one optimizer can search all of them.

## 1. Match a method to a stated task

| Parameter family | Candidate approaches | Conditions to check |
|---|---|---|
| continuous MPC costs | gradients, Bayesian/derivative-free search | valid sensitivities or affordable evaluations |
| horizon, solver, discrete modes | enumeration, structured/mixed-variable BO | integer/categorical semantics and execution cost |
| slack/penalty weights | constrained or differentiable search | feasibility meaning and active-set changes |
| hardware feedback gains | conservative commissioning, contextual/Safe BO when justified | safe seed, confidence assumptions, protection |
| covariance/kinematics | likelihood, EM, residual diagnostics, bilevel fitting | identifiability, correlations and supervision |
| mass/inertia/gain/friction/delay | physically constrained identification | excitation, confounding and independent prediction data |

A single black-box optimizer can be a valid baseline. Its name is not proof of quality or inadequacy. Continuous CMA-ES does not automatically handle categorical choices; encodings or structured alternatives need explicit design. Gradients are not automatically more efficient or useful for every parameter.

## 2. Objective, sensitivities and parameterization

An MPC's internal short-horizon objective is not automatically the engineering closed-loop objective. Specify tracking, effort, slew, excursions, recovery, constraint/failure behavior and execution cost with physical units or explicit normalization. An early-terminated trial cannot be ranked as excellent merely because its accumulated error is short.

DiffTune-MPC is a reference for differentiating closed-loop performance with respect to MPC parameters. Check solution-map regularity, active-set transitions, model fidelity and numerical convergence. Differentiating a few iterations is not automatically differentiating the converged optimum. Hybrid modes and discrete decisions require separate treatment.

Positive scalar/diagonal weights may use exponentials or softplus; neither is mandatory. A full matrix needs an appropriate constrained factor or other matrix parameterization. L Lᵀ alone is semidefinite, not necessarily definite. Scaling every cost term equally leaves the exact optimizer unchanged, creating redundant parameterizations; scaling only some terms or changing numerical conditioning can change practical results.

When optimizing covariance likelihoods, include their normalization/log-determinant terms. Minimizing only residuals weighted by the inverse of freely enlarging covariance permits trivial inflation. The local CEM RMSE/NIS/NEES score is an educational heuristic, not a complete maximum-likelihood method or formal consistency test.

## 3. Safe hardware exploration is conditional

The cited legged Safe BO study demonstrates its method under a particular safe-seed, confidence and operating contract. It does not make Safe BO mandatory for every robot or replace independently enforced protective limits. Missing observability, bad actuator identification or an invalid model is not repaired by the optimizer's name.

Hardware trials require a platform-specific admission policy and verified stop/recovery behavior. 'Safe' in a method title is not unconditional authorization to explore a real robot. This lab has not commissioned hardware.

## 4. Calibration and co-design references

LegBiCal studies covariance and kinematics in a bilevel estimation problem. Residual-based adaptive filtering studies online covariance changes. Their findings depend on data, supervision, excitation and models; no conference-acceptance claim is made here without verified proceedings.

ContEst (`2609.36090`) was recoverable at the official indexed-abstract level in this review. Its abstract describes envelope-theorem design gradients and an approximate nonlinear EKF/MPC extension. Full-text-specific scalability/uniqueness/implementation guarantees were not independently verified and are not treated as authority. It is not reproduced by the local CEM co-tuner.

Sequential identification/calibration/design is a useful diagnostic starting point; joint design may be appropriate for coupled problems. This is an iterative practice, not a theorem that MHE or online adaptation must always be added.

## 5. Selection sets and final-test reuse

Use training data for fitting and validation for choices. Freeze a final test before measuring it. Once that result guides a new design, it becomes development evidence for subsequent changes, not an untouched final test forever.

The legacy commissioning code compares both validation and a set named test when deciding admission. That is its actual contract; repeated use does not support unbiased final-test claims. Different seeds alone do not eliminate shared model/distribution effects. A separately frozen prospective terminal manifest is declared evidence; repeated use after implementation changes is a regression on that known set.

Report failed/rejected plans, common-prefix comparisons after early exits, and full-duration comparisons only for both-completed pairs with exclusions. Zero failures in a scripted grid is not a probabilistic safety guarantee. Binomial confidence statements need an appropriate independent sampling model and a stated trial unit.

## 6. Local implementation scope

`src/commissioning.js` contains constrained derivative-free plant fitting, actuator fitting, heuristic covariance calibration, LQR CEM, a small structured MPC search and joint CEM parameters. These are not DiffTune, Safe BO or LegBiCal reproductions. Plant fitting uses simulated truth, actuator fitting uses realized force and baseline covariance can use injected noise. A sensor-only deployment path is still a separate requirement.

The browser retains MuJoCo WASM and quadprog. Riccati terminal cost is an explicit option, not a new optimizer. Native acados/HPIPM experiments are separate research work until integrated; supported library features do not imply browser availability.

## Primary references

- DiffTune-MPC: https://arxiv.org/abs/2312.11384
- Legged Safe BO: https://proceedings.mlr.press/v229/widmer23a.html
- LegBiCal: https://arxiv.org/html/2510.11539v1
- Adaptive covariance: https://arxiv.org/abs/2608.02316
- ContEst (abstract verified only): https://arxiv.org/abs/2609.36090
- Native sensitivities and solver features: https://docs.acados.org/features/

Use [Implementation boundaries](IMPLEMENTATION_BOUNDARIES.md), [Model hierarchy](MODEL_HIERARCHY.md) and [Observers](OBSERVERS.md) to interpret a local result before transfer.
