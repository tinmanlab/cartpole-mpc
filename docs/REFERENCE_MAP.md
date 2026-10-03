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

This is a better Figure reference for Q_e/R_e + kinematic calibration than hand tuning.

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

Paper:
- https://arxiv.org/abs/2609.36090

Core idea:
- joint design of actuation, sensing and estimation,
- exact envelope-theorem gradients in convex LQG/H-infinity regimes,
- approximate EKF-MPC extension for nonlinear constrained systems.

Important limitation for humanoids:
- the paper itself notes scaling and approximation limits in the nonlinear information-state formulation,
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

These papers modify different interfaces. Do not group them under one generic "learned observer" label.

## Native dynamics / geometry

For a humanoid:
- rigid-body dynamics and derivatives: Pinocchio or equivalent,
- collision geometry: native rigid-body geometry stack,
- simulator: MuJoCo or another independently validated plant,
- actuator: platform-specific identified actuator model,
- OCP: OCS2/acados as appropriate.

The CartPole repository owns the **commissioning logic and diagnostic questions**, not the production humanoid solver.
