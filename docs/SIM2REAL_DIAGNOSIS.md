# Sim2Real diagnosis: isolate the failure before changing the controller

> HISTORICAL snapshot at its stated revision/configuration. Numeric results and prior implementation descriptions below are preserved as evidence, not current runtime claims. Current semantics are owned by IMPLEMENTATION_BOUNDARIES.md, MODEL_HIERARCHY.md, CONTROLLERS.md, OBSERVERS.md and MUJOCO_WASM_RUNTIME.md.

A simulation-to-real failure is not a single problem called "reality gap".

The purpose of this repository is to keep the failure axes separable enough that a complex robot project can ask:

> Which assumption failed, and which experiment would falsify it?

## Diagnostic ladder

Always start from the smallest pair that differs by one assumption.

| Question | A arm | B arm | If B fails while A passes |
|---|---|---|---|
| Is the control law itself viable? | oracle state + nominal plant | same | controller/formulation issue |
| Does state estimation break it? | oracle state | estimator in loop | estimator / observability / covariance |
| Does model mismatch break it? | nominal model | parameter mismatch | SysID / robustification |
| Does sensor noise break it? | clean measurement | noisy measurement | filtering / sensor model |
| Does slowly varying bias break it? | white noise | bias random walk | bias state / calibration / observability |
| Does actuator bandwidth break it? | ideal command | lag + gain error | actuator model / inner loop |
| Does transport delay break it? | no delay | delayed command | delay compensation / timing |
| Does one bad measurement break it? | Gaussian noise | deterministic glitch | outlier rejection / robust update |
| Do combined effects interact? | single factors | sim2real stack | coupling / margin problem |
| Does computation break it? | offline/unbounded | deadline/scheduler | solver/runtime system |

Do not start with the combined stack. Combined randomization is an admission test, not a diagnosis.

## Executable CartPole ablations

The live lab has these scenarios.

- **Nominal** — shared nominal model, Gaussian sensor noise only.
- **Model mismatch** — mass, pole mass, length, friction mismatch.
- **Sensor noise** — stronger white measurement noise.
- **Sensor bias random walk** — slowly varying position/angle bias.
- **Actuator lag + gain error** — command differs from applied command.
- **Command latency** — two controller ticks of delayed actuation.
- **Measurement glitch** — deterministic position outlier.
- **Mixed model + sensor** — model mismatch plus measurement noise.
- **Sim2real stack** — hidden episode-level model, noise, bias, delay, lag and gain mismatch.

The commissioning receipt includes a single-factor ablation for both:
- LQR + EKF,
- Full NMPC + EKF.

For each factor it records:
- failures,
- state-estimation RMSE,
- maximum cart excursion,
- RMS control,
- command-to-applied-command mismatch,
- solver time,
- delta from nominal.

This is evidence for *localization*, not an algorithm leaderboard.

## Failure taxonomy

### 1. Model-structure failure

Symptoms:
- all parameter fits remain poor,
- residuals are state-dependent,
- controller retuning moves the failure but does not remove it.

Examples on larger robots:
- rigid model used where compliance matters,
- missing actuator state,
- missing contact mode,
- rigid contact used for deformable terrain.

Action:
- change the model class before tuning weights.

### 2. Parameter-identification failure

Symptoms:
- one-step model residual changes strongly with mass/friction/latency,
- different parameter sets explain the same dataset,
- tuned controller compensates for wrong physical parameters.

Action:
- persistent excitation,
- physically constrained SysID,
- held-out trajectory validation,
- inspect identifiability / parameter correlation.

Humanoid mapping:
- link mass / CoM / inertia,
- kinematic offsets,
- actuator constants,
- reflected inertia,
- friction / compliance,
- command and sensing latency.

### 3. Observability / estimator-structure failure

Symptoms:
- oracle-state controller succeeds,
- estimator-in-loop controller fails,
- covariance grows in unobservable directions,
- innovation is biased or correlated.

Action:
- observability analysis,
- bias augmentation,
- contact/exteroceptive measurement,
- EKF vs invariant filter vs MHE according to model structure.

No choice of Q_e/R_e can make a fundamentally unobservable direction observable.

### 4. Estimator-calibration failure

Symptoms:
- state RMSE is acceptable but P is overconfident,
- NIS/NEES are inconsistent,
- the filter diverges after a disturbance.

Action:
- identify Q_e/R_e from data,
- bilevel covariance/kinematic calibration,
- validate NIS/NEES on held-out trajectories,
- use online covariance adaptation only for time-varying uncertainty.

### 5. Controller-objective failure

Symptoms:
- the controlled variable is stable while an unpenalized quantity drifts,
- the optimizer "does exactly what was asked" but the robot fails.

CartPole example:
- pole recovers,
- cart hits the track boundary.

Action:
- fix the engineering objective and constraints,
- do not hide a safety requirement only by increasing a weight.

### 6. Constraint / feasibility failure

Symptoms:
- good nominal tracking,
- state or actuator limits are crossed under recovery.

Action:
- explicit state/input/path constraints,
- soft-vs-hard constraint policy,
- terminal treatment,
- infeasibility/fallback behavior.

Humanoid mapping:
- joint limits,
- torque-speed envelope,
- friction cone,
- unilateral contact,
- CoP/support region,
- swing clearance,
- self/environment collision,
- thermal/power limits.

### 7. Actuator-model failure

Symptoms:
- command and applied effort differ,
- high gains help in ideal simulation but destabilize the real platform,
- saturation or lag appears around aggressive motion.

Action:
- identify low-level gain/bandwidth,
- torque-speed/current constraints,
- reflected inertia,
- friction/compliance/backlash,
- thermal derating when material.

### 8. Timing / transport failure

Symptoms:
- offline or single-threaded solver passes,
- real-time loop fails,
- behavior changes with CPU scheduling or telemetry load.

Action:
- measure command-to-state latency,
- jitter distribution, not only mean,
- solver p50/p95/p99 and deadline misses,
- asynchronous perception/control contracts.

### 9. Contact / hybrid-mode failure

CartPole cannot reproduce this faithfully.

Humanoid symptoms:
- stance FK update is good in one phase and harmful in another,
- slip or partial contact corrupts state estimation,
- controller uses the wrong contact mode.

Action:
- keep separate:
  1. contact/mode hypothesis,
  2. measurement construction,
  3. measurement reliability/covariance,
  4. estimator update,
  5. plant truth used only for scoring.

### 10. Tuning / overfit failure

Symptoms:
- training objective improves,
- held-out validation or test becomes worse.

Action:
- reject the candidate.
- widen training conditions only after the failure has been understood.

The current commissioning receipt intentionally contains this failure for controller tuning.

## Robot-scale diagnostic order

For Figure-like humanoids:

    descriptor / frame / joint-order sanity
    → kinematic + inertial + actuator ID
    → sensor calibration
    → oracle-state controller admission
    → estimator calibration
    → estimator-in-loop admission
    → explicit constraints
    → actuator dynamics
    → latency / jitter
    → contact uncertainty
    → single-factor stress
    → combined held-out stress
    → hardware shadow replay
    → low-energy safe trials
    → safe residual tuning
    → wider task envelope

A later stage never erases evidence from the earlier stages.

## Important rule

Domain randomization is not a substitute for diagnosis.

Use randomization to test robustness after:
- the nominal model is identified,
- the uncertainty family is measured or justified,
- single-factor failures are understood.

Otherwise a successful policy/controller can hide a wrong model, and a failed one does not tell you why.

## Advanced fault signatures

Do not collapse the following into one generic reality-gap bucket:

| Signature | CartPole probe | Robot-scale interpretation |
|---|---|---|
| innovation autocorrelation | AR(1) colored sensor noise | white-noise assumption is wrong; model correlation or augment the noise state |
| stale/frozen packets | dropout hold-last | timestamp/freshness/transport fault, not Gaussian R tuning |
| plausible but stuck measurement | stuck sensor | fault detection/isolation and degraded sensing |
| command/applied mismatch with irregular timing | jitter | transport/scheduler distribution and delay compensation |
| speed-dependent force authority | torque-speed envelope | motor voltage/current/back-EMF envelope belongs in constraints/model |
| slowly declining authority | thermal derating | thermal/current/power state and long-duration acceptance |
| trajectory changes with integrator refinement | discretization probe | numerical model error; validate timestep/integrator before retuning gains |

The educational scenarios expose these mechanisms separately. A real robot should replace each with synchronized measured telemetry rather than reusing the CartPole constants.

## Runtime safety/degraded mode

The executable NMPC supervisor monitors its predicted CartPole trajectory and switches to a local LQR backup when position/angle margins are exceeded. This demonstrates architectural separation between performance control and a backup layer. It is not a CBF, reachability proof, viability kernel, terminal invariant set, or hardware safety guarantee. Figure should use native constraints, verified fallback behavior and hardware interlocks appropriate to the platform.
