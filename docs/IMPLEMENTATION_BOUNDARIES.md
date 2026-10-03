# Implementation boundaries

This file exists to prevent educational analogues from silently becoming capability claims.

| Item | In this repository |
|---|---|
| PID | actual live CartPole controller |
| LQR | actual Riccati feedback |
| Linear MPC | actual finite-horizon receding controller |
| Centroidal-style MPC | actual reduced CoM planner plus downstream stabilizer |
| Full nonlinear NMPC | actual nonlinear receding-horizon local iterative solver |
| PPO | actual frozen CartPole actor |
| KF | actual linear Kalman filter |
| EKF | actual nonlinear prediction and covariance propagation |
| InEKF page | theory + SO(2) runtime bridge, not full humanoid InEKF |
| InNKF runtime | trained residual MLP analogue, not paper architecture reproduction |
| CoCo/FOCUS runtime | adaptive-R structural bridge, not learned paper model |

## Figure / wb_humanoid_mpc provenance

The controller hierarchy was derived by reading:

- tinmanlab/figure evidence for the pinned model-based comparator,
- manumerous/wb_humanoid_mpc at ab5dbfd1df07a33258ce7f2998f5c315f1fcc8b9.

The source shows two distinct formulations.

Centroidal model:
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

## What not to infer

CartPole success does not establish:
- humanoid walking performance,
- hardware estimator robustness,
- real contact handling,
- actuator safety,
- OCS2 timing parity,
- SOTA superiority.
