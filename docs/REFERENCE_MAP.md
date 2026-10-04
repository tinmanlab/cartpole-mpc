# Reference and native-authority map

This page distinguishes **concept references** from the software that should own a robot-scale implementation.

## Nonlinear / switched optimal control

### OCS2

Official documentation:
- https://leggedrobotics.github.io/ocs2/
- https://leggedrobotics.github.io/ocs2/overview.html
- https://leggedrobotics.github.io/ocs2/optimal_control_modules.html
- https://leggedrobotics.github.io/ocs2/from_urdf_to_ocp.html

Relevant capabilities:
- iLQR / SLQ,
- multiple-shooting SQP,
- interior-point and SLP variants,
- switched-system mode schedules and jump maps,
- nonlinear dynamics,
- hard/soft path constraints,
- automatic differentiation,
- URDF/Pinocchio integration.

Use OCS2 when the Figure problem is naturally a switched/nonlinear rigid-body OCP. Do not recreate this stack in JavaScript.

### acados

Official documentation:
- https://docs.acados.org/
- https://docs.acados.org/features/
- https://docs.acados.org/problem_formulation/

Relevant capabilities:
- NMPC and MHE,
- OCP-structured SQP,
- SQP-RTI / AS-RTI,
- multiple shooting,
- nonlinear constraints,
- first/second-order integration sensitivities,
- forward/adjoint solution sensitivities for differentiable MPC,
- solver residual and timing diagnostics.

This makes acados a strong native authority for experiments involving RTI, differentiable tuning, MHE/NMHE, and embedded real-time deployment.

## Model hierarchy

OCS2 explicitly distinguishes kinematic, dynamic, and centroidal models. For legged robots, centroidal dynamics are a deliberate reduced-order model rather than a synonym for full rigid-body dynamics.

For Figure-like work the model choice should be an explicit experimental variable:

    centroidal/reduced
      vs
    full-order rigid-body

and not a hidden implementation detail.

## MPC auto-tuning

### DiffTune-MPC

Paper:
- https://arxiv.org/abs/2312.11384

Core idea:
- tune MPC cost parameters against a long-horizon **closed-loop** objective,
- differentiate the MPC solution through KKT/implicit sensitivities,
- includes nonlinear MPC solved with SQP.

Use it as a reference for smooth continuous MPC-weight tuning. Do not use it as the only method for discrete solver/horizon choices or unsafe hardware exploration.

## Safe hardware tuning

### Tuning Legged Locomotion Controllers via Safe Bayesian Optimization

Paper:
- https://arxiv.org/abs/2306.07092

Key relevance:
- tunes model-based legged controller feedback gains,
- black-box objective and safety constraints,
- contextual tuning across gait parameters,
- hardware demonstration on Unitree Go1,
- 50 learning steps reported for hardware gait tuning.

Use Safe BO only after a safe seed exists and structural model/estimator problems have been addressed.

## Estimator calibration

### Simultaneous Calibration of Noise Covariance and Kinematics for State Estimation of Legged Robots via Bi-level Optimization

Paper:
- https://arxiv.org/abs/2510.11539
- code: https://github.com/DLinC3/LegBiCal

Core idea:
- outer variables include covariance and kinematic parameters,
- lower level solves a MAP/full-information estimator,
- differentiate through the estimator,
- enforce positive-definite covariance constraints,
- validate on quadrupedal and bipedal/legged platforms and real hardware.

This is a relevant covariance/kinematic calibration reference; benefit over another method must be measured for the actual data and estimator. Conference placement is not asserted here.

## Online covariance adaptation

### Residual-Based Adaptive Kalman Filtering for Legged Robot State Estimation

Paper:
- https://arxiv.org/abs/2608.02316

Core idea:
- adapt InEKF Q/R from innovation and residual statistics,
- real Unitree Go2 indoor/outdoor data,
- the reported ablation found R adaptation most useful for that setup.

Use online adaptation around a calibrated estimator. It does not make offline calibration or observability analysis unnecessary.

## Control-estimation co-design

### ContEst

Verification level: official indexed abstract recovered; full text and code were not independently reproduced in this audit. A failed page fetch alone does not establish that a paper is nonexistent.

Paper:
- https://arxiv.org/abs/2609.36090

Core idea:
- joint design of actuation, sensing and estimation,
- exact envelope-theorem gradients in convex LQG/H-infinity regimes,
- approximate EKF-MPC extension for nonlinear constrained systems.

Important limitation for humanoids:
- nonlinear extension and detailed scalability/guarantee claims need a full-text and implementation check before adoption,
- therefore use it as a research direction after staged commissioning, not as a reason to collapse every Figure subsystem into one optimizer.

## Invariant and learned legged estimation lineage

- Hartley et al., Contact-Aided Invariant EKF:
  https://arxiv.org/abs/1904.09251
- learned contact events / Lin:
  https://proceedings.mlr.press/v164/lin22b.html
- neural measurement network / Youm:
  https://arxiv.org/abs/2402.00366
- InNKF:
  https://arxiv.org/abs/2503.00344
- CoCo-InEKF:
  https://arxiv.org/abs/2605.15122
- FOCUS:
  https://arxiv.org/abs/2609.02222

These papers modify different interfaces. FOCUS is verified at official abstract level here; full-text network widths, thresholds and scale formulas were not rechecked and were removed from the active lesson. CoCo/InNKF and EKF+MHE were inspected at original HTML text level. No cited robot/network benchmark is locally reproduced merely by listing the source.

## Native dynamics / geometry

For a humanoid:
- rigid-body dynamics and derivatives: Pinocchio or equivalent,
- collision geometry: native rigid-body geometry stack,
- simulator: MuJoCo or another independently validated plant,
- actuator: platform-specific identified actuator model,
- OCP: OCS2/acados as appropriate.

The CartPole repository owns the **commissioning logic and diagnostic questions**, not the production humanoid solver.

## General teaching references and scope

- LQR and finite-horizon/local nonlinear use: https://underactuated.mit.edu/lqr.html
- OCP formulation, RTI phases, soft constraints and MHE examples: https://docs.acados.org/features/
- Convex input/state-constrained MPC: https://osqp.org/docs/examples/mpc.html
- KF reference implementation: https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html
- PPO original method: https://arxiv.org/abs/1707.06347
- EKF+MHE paper: https://arxiv.org/html/2405.20567v1

Research capabilities, a local optional reproduction and a browser-deployed implementation are separate states. Pinocchio is dynamics/kinematics/derivatives software, not an OCP optimizer. Native acados/HPIPM experiments in a separate unmerged research change do not make the deployed browser run acados. See each source's actual inspected content rather than treating paper titles, author conclusions or prior assistant reports as universal guarantees.
