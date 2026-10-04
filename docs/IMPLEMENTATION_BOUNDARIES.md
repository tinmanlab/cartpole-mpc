# Implementation boundaries

This file prevents educational analogues from silently becoming capability claims.

| Item | In this repository | Do not call it |
|---|---|---|
| PID | actual live cascaded CartPole feedback | humanoid low-level controller |
| LQR | actual DARE/Riccati state feedback | constrained optimal controller |
| Linear MPC | actual N=30 input-box QP using upstream quadprog, checked against OSQP | OSQP running in the browser / universal robot MPC |
| Constrained MPC | explicit input and nominal-prediction world-position constraints with KKT checks | robust true-plant invariant-set or hardware safety guarantee |
| Browser plant | official MuJoCo WASM + canonical uniform-rod MJCF | arbitrary humanoid contact dynamics / calibrated hardware asset |
| Scenario-risk MPC | actual finite 3-model ensemble objective with mean + worst-model risk term | tube/min-max/chance MPC guarantee |
| State-aware Linear MPC | actual soft cart-position penalty in horizon | hard state-constrained MPC / feasibility guarantee |
| LTV MPC / RTI bridge | actual one-update successive-linearization controller | acados/OCS2 SQP-RTI |
| Centroidal-style MPC | actual N=32 reduced CoM planner + downstream stabilizer | OCS2 centroidal port |
| Full nonlinear NMPC | actual N=30 single-shooting iLQR-style NMPC | OCS2 multiple-shooting SQP |
| PPO | actual frozen CartPole actor, deterministic evaluation | newly trained policy for this lab |
| KF | actual linear Kalman filter | hardware-tuned estimator |
| EKF | actual nonlinear mean + numerical-Jacobian covariance propagation | InEKF |
| Shooting MHE | actual recent-window nonlinear single-shooting estimate | constrained sparse MHE/NMHE |
| SO(2) error bridge | wrapped-angle EKF measurement handling | Hartley contact-aided InEKF |
| InNKF-style runtime | trained output-only residual MLP | paper TCN/SE2(3) reproduction |
| CoCo page | paper-faithful conceptual explanation only | Adaptive-R implementation of CoCo |
| Adaptive-R runtime | innovation-based observation-R modulation | learned FOCUS or CoCo |
| FOCUS page | paper equations + Adaptive-R structural bridge | FOCUS reproduction |

## Timestamp and covariance boundary

One trace row corresponds to one common post-step time:

    u_k -> x_(k+1) -> y_(k+1) -> x_hat_(k+1)

Metrics compare x_(k+1) with x_hat_(k+1).

For the residual observer:
- baseP belongs to the base EKF,
- corrected output P is unavailable,
- the UI must not draw baseP as confidence bounds for the corrected state.

## Figure / wb_humanoid_mpc provenance

The controller hierarchy was derived by reading:
- tinmanlab/figure evidence for the pinned model-based comparator,
- manumerous/wb_humanoid_mpc at ab5dbfd1df07a33258ce7f2998f5c315f1fcc8b9.

Centroidal model in that source:
- state: normalized centroidal momentum + base pose + joint angles,
- input: left/right contact wrench + joint velocities,
- gait/contact schedule and contact constraints,
- inverse-dynamics realization after MPC policy evaluation.

Full-order model:
- state: base/joint positions + base/joint velocities,
- input: left/right contact wrench + joint accelerations,
- full acceleration dynamics in the OCP,
- joint-torque cost and contact/swing constraints.

No OCS2, Pinocchio, or wb_humanoid_mpc source code is copied into this repository.

## Local task-contract boundary

The common CartPole evaluation is a teaching contract, not an official benchmark.

In particular:
- the lab failure angle is wider than the original CartPole PPO Studio termination threshold,
- the frozen PPO actor may be driven by observer outputs it was not trained with,
- solver timing is host/browser dependent,
- a lower RMSE in one fixed scenario is not a general controller or estimator ranking.

## What not to infer

CartPole success does not establish:
- humanoid walking performance,
- hardware estimator robustness,
- real contact handling,
- actuator safety,
- OCS2 timing parity,
- SOTA superiority.

## Commissioning boundaries

| Item | In this repository | Not claimed |
|---|---|---|
| System identification | constrained CartPole mc/mp/l/friction fit | full robot inertial/actuator ID |
| Estimator calibration | Q_e/R_e educational CEM with NIS/NEES penalty | statistically complete hardware calibration |
| Controller tuning | LQR Q_c/R_c educational CEM + held-out reject gate | SOTA differentiable/Safe-BO tuner |
| Sim2real stack | hidden model/noise/bias/delay/lag/gain mismatch | humanoid contact/thermal/network completeness |
| Robust MPC beyond finite scenarios | taxonomy/coverage page | executable tube/min-max/chance MPC guarantee |

The tuning algorithms are examples of the commissioning contract, not the recommended solver for every robot.

Runtime engine, asset identity, exact model conventions and executable acceptance tests are owned by [MuJoCo WASM runtime](MUJOCO_WASM_RUNTIME.md).
