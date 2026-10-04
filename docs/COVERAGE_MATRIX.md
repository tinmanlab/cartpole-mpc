# Coverage matrix

Legend:

- **EXECUTABLE** — implemented and exercised in this repository.
- **BRIDGE** — executable simplified analogue; claim boundary is explicit.
- **CONCEPT** — taught and mapped to the correct robot-scale method, not implemented.
- **EXTERNAL** — should be handled by a mature external/native stack for complex robots.

| Area | Status | CartPole coverage | Complex-robot path |
|---|---|---|---|
| MuJoCo WASM plant / MJCF | EXECUTABLE | official pinned runtime, one canonical explicit-inertia MJCF, native replay and geometry checks | robot-specific validated MJCF and declared full simulator state |
| PID / low-level feedback | EXECUTABLE | cascaded feedback | joint torque/position feedback |
| LQR | EXECUTABLE | discrete shared-plant Jacobian | local linear controller/terminal model |
| linear constrained MPC | EXECUTABLE | input-box constrained horizon | sparse QP MPC |
| linear state constraints | EXECUTABLE | hard world-rail QP plus separately labelled soft-penalty comparison | nonlinear path/contact constraints remain a native OCP task |
| LTV / successive linearization | BRIDGE | one RTI-style update/tick | SQP-RTI / multiple shooting |
| reduced-order MPC | BRIDGE | CoM planner + lower layer | centroidal momentum/contact-wrench MPC |
| nonlinear MPC | EXECUTABLE | 4-state single-shooting iLQR-style | sparse full-order NMPC |
| robust/stochastic MPC | BRIDGE | executable finite-model scenario-risk MPC + taxonomy; no formal robustness guarantee | tube/min-max/scenario/chance MPC |
| safety filter / CBF / backup | BRIDGE | executable NMPC prediction monitor + LQR backup; no formal guarantee | CBF/reachability/verified backup + hardware interlock as justified |
| hybrid/switched contact | CONCEPT | no fake foot variables | switched/contact-mode OCP |
| terminal invariant set | CONCEPT | not implemented | task-specific MPC stability design |
| recursive KF | EXECUTABLE | linear KF | sensor-specific KF/ESKF |
| EKF | EXECUTABLE | nonlinear mean/Jacobian | floating-base nonlinear filter |
| UKF | EXECUTABLE | sigma-point nonlinear Gaussian filter | Jacobian-free nonlinear Gaussian filtering when justified |
| invariant EKF | BRIDGE | SO(2) error geometry only | SO(3)/SE(3)/SE_2(3) InEKF |
| MHE | BRIDGE | nonlinear single-shooting window | constrained sparse MHE/NMHE |
| factor graph / smoothing | CONCEPT | not implemented | VIO/VILO/factor graph when needed |
| estimator consistency | EXECUTABLE offline | NIS/NEES commissioning metric | statistical calibration over held-out data |
| adaptive covariance | BRIDGE | innovation-based observation R | residual/contact-reliability adaptation |
| learned residual estimator | BRIDGE | output-only residual MLP | InNKF-style learned correction if justified |
| learned contact/reliability | CONCEPT | Lin/Youm/CoCo/FOCUS pages | robot-specific learned measurement authority |
| system identification | EXECUTABLE | simulated truth-based mc/mp/l/friction + realized-force actuator gain/lag/delay, held-out rollout | inertial/kinematic/actuator/latency ID |
| identifiability diagnostic | EXECUTABLE | residual sensitivity correlation | excitation design / Fisher-information analysis |
| estimator calibration | EXECUTABLE offline | Q_e/R_e scale + NIS/NEES mean/coverage diagnostics | bilevel/likelihood calibration |
| controller tuning | EXECUTABLE offline | LQR CEM + structured Linear-MPC horizon/cost search with held-out accept/reject gates | DiffTune/BO/Safe BO as appropriate |
| control-estimation co-design | BRIDGE | low-dimensional closed-loop CEM co-tuning + held-out gate | ContEst/differentiable/bilevel co-design when tractable |
| model validity envelope | EXECUTABLE offline | fixed-angle sweep across LQR/Linear/State-aware/LTV/Full-NMPC | robot-specific operating-envelope evidence |
| train/validation/test split | EXECUTABLE | disjoint conditions/seeds + accept/reject gate | mandatory deployment evidence |
| sensor noise / bias / outlier | EXECUTABLE | white noise, bias random walk, deterministic glitch | measured sensor models + robust update |
| colored noise / packet faults | EXECUTABLE | AR(1) colored noise, dropout metadata with prediction-only assimilation, stuck sensor + fault metadata | timestamp-aware fault detection, colored-noise/robust sensor model |
| actuator lag/gain | EXECUTABLE | isolated first-order lag/gain scenario + combined stack | identified motor/drive model |
| command delay / jitter | EXECUTABLE | isolated fixed delay + stochastic command hold jitter | measured latency/jitter distribution + delay-aware prediction |
| timing/deadline | EXECUTABLE diagnostic | solver-call timing plus live synchronous compute age; not sensor-to-hardware timing | scheduler/solver deadline acceptance |
| discretization sensitivity | EXECUTABLE offline | 1/2/4/8/16 substep convergence probe | integrator/timestep convergence and error budget |
| diagnosis triage | EXECUTABLE offline | deterministic evidence → model/estimator/constraint/actuator/timing/tuning findings | robot-scale triage using native telemetry |
| friction/model mismatch | EXECUTABLE | hidden parameter family | identified uncertainty bracket |
| single-factor sim2real ablation | EXECUTABLE | model/noise/bias/actuator/latency/outlier before combined stack | matched robot-scale ablation campaign |
| torque-speed/current loop | BRIDGE | speed-dependent force envelope; no electrical current loop | actuator-native voltage/current/torque-speed model |
| thermal/power derating | BRIDGE | history-dependent scalar thermal state and force derating | actuator thermal/current/power model |
| contact/slip | EXTERNAL | CartPole cannot represent it faithfully | humanoid/legged plant + estimator |
| self/environment collision | EXTERNAL | absent | rigid-body geometry/OCP |
| exteroceptive perception | EXTERNAL | absent | VIO/LiDAR/depth pipeline |
| safe hardware tuning | CONCEPT | no hardware | Safe BO / governed commissioning |
| hardware verification | EXTERNAL | no hardware claim | platform-specific campaign |

## Admission rule

A feature is not "covered" merely because a page mentions it.

For reuse in Figure or another robot, require:
1. a native model/solver that represents the relevant physics,
2. measurable parameters or a declared uncertainty set,
3. a matched estimator/controller signal contract,
4. held-out evidence,
5. failure-mode and timing diagnostics,
6. explicit claim boundaries.

The CartPole lab should stay small enough that every executable item can be audited numerically.
