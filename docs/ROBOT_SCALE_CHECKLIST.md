# Robot-scale commissioning checklist

This checklist exists because a small CartPole can make important robot-scale problems disappear.

A complex robot should not inherit CartPole equations. It should inherit the questions.

## A. State and model definition

- [ ] Frame convention is explicit and tested.
- [ ] Quaternion / SO(3) convention is explicit.
- [ ] Joint ordering is identical across model, estimator, controller, logs and hardware.
- [ ] Units are checked.
- [ ] Mass, center of mass and inertia are physically plausible.
- [ ] Kinematic offsets are calibrated.
- [ ] Floating-base convention is explicit.
- [ ] Contact frames and wrench sign conventions are tested.
- [ ] Gravity convention is identical across stack.
- [ ] Model parameters have an identified value **and** an uncertainty bracket.
- [ ] Structural model limitations are listed.

## B. Actuation

- [ ] Command variable is clear: current / torque / velocity / position.
- [ ] Command-to-applied effort is measured.
- [ ] Saturation limits are measured.
- [ ] Torque-speed or force-velocity envelope is represented when material.
- [ ] Current-loop / low-level bandwidth is measured.
- [ ] Reflected inertia is represented.
- [ ] Friction is identified over direction/speed/load where necessary.
- [ ] Compliance/backlash is represented when it changes closed-loop behavior.
- [ ] Command latency and jitter are measured.
- [ ] Thermal/power derating is represented if sustained operation matters.
- [ ] Emergency/fallback actuator behavior is known.

## C. Sensor and timing contract

- [ ] Every estimator input exists on real hardware.
- [ ] Simulator-only ground truth is scorer-only.
- [ ] Sensor sampling rates are measured.
- [ ] Timestamp source and clock synchronization are verified.
- [ ] Time offsets/extrinsics are calibrated.
- [ ] White noise is distinguished from bias/random walk.
- [ ] Outliers/dropouts are modeled separately from Gaussian noise.
- [ ] Saturation/quantization/resolution are represented when material.
- [ ] Processing/perception latency distribution is measured.

## D. Observability and estimation

Before tuning Q_e/R_e, ask whether the state is observable.

- [ ] Local observability analysis is performed where applicable.
- [ ] Gauge freedoms/unobservable directions are documented.
- [ ] Bias states are added when bias is observable enough to estimate.
- [ ] Contact assumptions are separated from measurement reliability.
- [ ] Innovation/residual whiteness is inspected.
- [ ] NIS/NEES or equivalent consistency diagnostics are checked.
- [ ] Covariance is not attached to a learned-corrected output unless calibrated for that output.

Estimator family selection:

| Situation | Candidate |
|---|---|
| linear Gaussian | KF |
| moderate smooth nonlinearity | EKF / ESKF |
| useful Lie-group / group-affine structure | InEKF |
| nonlinear propagation where Jacobians are undesirable | UKF / sigma-point filter |
| explicit constraints / recent-window re-optimization | MHE / NMHE |
| long-horizon asynchronous multi-sensor smoothing | factor graph / smoothing |
| strongly non-Gaussian or multimodal belief | particle / mixture methods |

Do not choose a more complicated estimator merely because it is newer.

## E. Contact and hybrid dynamics

For legged/humanoid systems:

- [ ] Contact mode hypothesis has its own interface.
- [ ] Planned contact and sensed contact are distinguishable.
- [ ] Measurement construction is separate from contact detection.
- [ ] Measurement covariance/reliability is continuous when appropriate.
- [ ] Slip and partial contact are represented.
- [ ] Unilateral contact is enforced.
- [ ] Friction constraints are enforced.
- [ ] Impact/reset dynamics are handled where relevant.
- [ ] Mode-transition timing uncertainty is stress-tested.

## F. Controller formulation

- [ ] Controlled outputs and engineering objective match.
- [ ] Safety requirements are not hidden only in cost weights.
- [ ] Input constraints are explicit.
- [ ] State/path constraints are explicit.
- [ ] Terminal cost/set assumptions are documented.
- [ ] Recovery behavior is tested outside nominal tracking.
- [ ] Estimator-in-loop behavior is tested.

Model hierarchy:

    fixed LTI
    → LTV / successive linearization
    → nonlinear OCP
    → hybrid/switched OCP

Reduced versus full-order:

    reduced/centroidal planner + lower-level realization
    versus
    full-order rigid-body optimal control

Both are valid architectures with different approximation and compute tradeoffs.

## G. Robustness and uncertainty

Nominal MPC is not robust MPC.

Possible tools:
- constraint tightening,
- tube MPC,
- min-max MPC,
- scenario MPC,
- chance constraints,
- risk-sensitive / CVaR objectives,
- disturbance observers,
- adaptive control,
- online parameter estimation.

Select the method according to the uncertainty structure; do not add all of them simultaneously.

## H. Safety layer and fallback

- [ ] A safe seed controller exists before online tuning.
- [ ] Constraint violation has a defined response.
- [ ] Solver infeasibility has a fallback.
- [ ] Deadline miss has a fallback.
- [ ] Estimator divergence has a fallback.
- [ ] Hardware emergency stop path is independent of high-level optimization.

Possible higher-level tools include:
- control barrier function safety filters,
- backup controllers,
- reachability/viability analysis,
- runtime monitors.

These are not implemented in CartPole-MPC and must not be inferred from it.

## I. MPC/NMPC numerical health

A mathematically correct OCP can still fail numerically.

- [ ] State/control scaling is sensible.
- [ ] Hessian/regularization conditioning is monitored.
- [ ] Warm start is validated.
- [ ] Line search / trust-region behavior is monitored.
- [ ] Primal/dual residuals are logged for production solvers.
- [ ] Constraint margins are logged.
- [ ] Infeasibility is distinguished from solver timeout.
- [ ] Iteration count and solve-time quantiles are logged.
- [ ] Deadline misses are measured.
- [ ] Multiple-shooting defects are monitored when applicable.

For humanoids, use native sparse solver diagnostics from OCS2/acados rather than recreating them in this browser lab.

## J. Identification and tuning

Order:

    model/actuator/latency ID
    → estimator calibration
    → nominal controller design
    → closed-loop tuning
    → held-out validation
    → safe hardware residual tuning
    → runtime adaptation

- [ ] Excitation is sufficient for identified parameters.
- [ ] Parameter sensitivity/correlation is checked.
- [ ] Identified model improves held-out rollout, not only training one-step loss.
- [ ] Tuner train/validation/test conditions are separated.
- [ ] Hyperparameters are transformed so physical constraints are respected (positive weights/covariances).
- [ ] Online tuning is safety constrained.

## K. Solver/model selection for humanoids

Use mature native components rather than scaling this repo:

- rigid-body dynamics/derivatives: Pinocchio or equivalent,
- switched/nonlinear OCP: OCS2 when appropriate,
- NMPC/MHE and RTI: acados when appropriate,
- simulation: MuJoCo / validated native simulator,
- hardware actuator model: platform-specific identified model.

CartPole-MPC is the commissioning logic reference, not the robot-scale solver authority.

## L. Evidence ladder

1. unit/math verification,
2. oracle-state nominal simulation,
3. estimator-only replay,
4. estimator-in-loop nominal,
5. one realism factor at a time,
6. combined held-out sim2real,
7. timing/scheduler stress,
8. recorded hardware shadow replay,
9. low-energy hardware,
10. wider task envelope.

Never skip directly from step 2 to step 9 and then tune until it works.

## Advanced sim2real failure checks

- [ ] Colored/correlated sensor noise is tested separately from white-noise covariance.
- [ ] Packet freshness/timestamps distinguish dropout, delay, and stuck sensors.
- [ ] Commanded and applied effort are synchronized and compared under jitter.
- [ ] Torque-speed/current/voltage envelopes are represented explicitly where material.
- [ ] Thermal/power derating is tested over sustained operation, not only short episodes.
- [ ] Integration timestep/integrator convergence is checked before gain retuning.
- [ ] UKF/MHE/factor-graph alternatives are selected because of structural need, not algorithm novelty.
- [ ] Joint control-estimation tuning is attempted only after model/estimator/controller blocks have independent evidence.
