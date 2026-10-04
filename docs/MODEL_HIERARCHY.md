# Model, formulation, transcription and solver: independent choices

This is a map of choices, not a ladder in which each entry universally improves on the previous one. The controller selected in the live UI is independent of the lesson being read. Concrete runtime implementations are classified in [Implementation boundaries](IMPLEMENTATION_BOUNDARIES.md).

## 1. Fixed LTI and time-varying linear models

The local upright model uses deviations about an equilibrium:

    e_(k+1) = A e_k + B delta_u_k

The lab constructs A/B from the same nominal discrete MuJoCo transition as the nonlinear prediction model. This provides local model consistency, not global accuracy away from upright. LQR, KF and Linear MPC use this linearization. An LTI dynamics model alone does not make an optimization problem convex: cost and constraints also matter.

An LTV model has A_k/B_k varying with time. It can be prescribed independently or obtained by linearizing a nonlinear model along a nominal trajectory. For the latter:

    A_k = df/dx at (xbar_k, ubar_k)
    B_k = df/du at (xbar_k, ubar_k)
    delta_x_(k+1) ≈ A_k delta_x_k + B_k delta_u_k + d_k
    d_k = f(xbar_k, ubar_k) - xbar_(k+1)

The affine defect d_k vanishes only when the nominal trajectory is dynamically feasible (with the stated discretization). A general trajectory linearization cannot silently omit it.

## 2. Nonlinear MPC is a problem formulation

A nonlinear OCP retains nonlinear dynamics and/or cost/constraints in its optimization problem. A dynamics example is:

    x_(k+1) = f(x_k, u_k)

SQP and iLQR-type methods use derivatives and local approximations; sampling-based or derivative-free methods need not construct those same local linear models. Therefore neither 'NMPC never linearizes' nor 'every NMPC algorithm repeatedly linearizes' is a general definition.

Finite-horizon LQR exists too. Receding-horizon execution and the chosen constrained optimization formulation distinguish the implemented MPC, not simply the presence of a finite horizon. Input-only and soft-constrained MPC are legitimate formulations; hard state constraints are an application choice.

## 3. Transcription, solver and execution strategy are different axes

| Axis | Examples | Does not by itself establish |
|---|---|---|
| Prediction model | LTI, LTV, nonlinear; reduced or full order | solver, convexity or performance |
| Constraint formulation | input/state/path, hard/slack/penalty, stochastic | true-plant safety under mismatch |
| Transcription | single shooting, multiple shooting, collocation | a particular optimizer |
| Numerical method | active-set QP, SQP, IPM, iLQR/DDP variants | correct model or real-time execution |
| Execution strategy | solve to a tolerance, limited iterations, RTI preparation/feedback | convergence or deadline certification |

Single shooting can be appropriate for large systems; dimension alone does not rule it out. Multiple shooting can expose useful sparsity and improve some conditioning/initialization properties, but adds state variables and continuity constraints. Select using the problem structure, derivatives, memory, constraints and measured execution cost.

The browser `ltv_mpc` is one iLQR-style local update with bounded forward line search. It is not acados/OCS2 constrained SQP-RTI. Its cost-decrease flag is `updateAccepted`; nonlinear optimality is not tested, so `converged` is null. The browser full nonlinear controller performs up to two local updates. Clipping its force does not reproduce an exact constrained QP or a box-DDP active-set solve.

## 4. Reduced versus full models

The local reduced controller plans only horizontal center of mass c and velocity c_dot. Its ideal continuous momentum relation is M*c_ddot=F_net. The discrete planner and downstream LQR are approximations; its planned force and applied LQR force differ. It supplies no theta/omega forecast, so no fabricated full-state ghost trajectory is displayed.

Humanoid centroidal formulations can retain momentum, CoM and different subsets of configuration/contact variables. There is no single mandatory state/input ordering. A particular pinned comparator's wrench/joint-velocity formulation is an example, not the definition of all centroidal MPC. Full-order formulations may use torques, accelerations, contact forces or other parameterizations with corresponding equations and constraints.

Pinocchio supplies rigid-body dynamics/kinematics and derivatives; it is not itself an OCP solver. OCS2 and acados supply different optimization facilities. The optional native research branch and a deployed browser implementation are separate scopes; a library's supported feature is not a feature reproduced in this lab.

## 5. Contact and hybrid dynamics

Contact transitions can introduce switching, impacts and uncertainty. Under ideal sticking contact, a constraint such as J_c(q)*v=0 applies. It does not cover slip, rolling, deformable contacts or unknown contact state without further modeling. Friction cones, unilateral normal forces and complementarity/mode schedules have different assumptions.

These dimensions are absent from the two-DOF slider/rod MJCF. CartPole success transfers diagnostic methods and interface contracts, not validated humanoid contact control or hardware authority.

## 6. Estimation choices

KF/EKF/UKF are recursive estimation choices; invariant-error geometry is another axis; MHE uses a moving optimization window. MHE can be linear or nonlinear, and neither MHE nor InEKF is a compulsory successor to EKF. Constraints, delayed sensing, noise model, observability and computational budget determine suitability.

The local MHE optimizes four initial-window state variables, uses an approximate EKF arrival prior and eight transitions (0.16 s). It has no process-noise trajectory decisions, hard state constraints or calibrated output covariance. See [Observers](OBSERVERS.md).

## References and use

- MIT LQR/finite-horizon/trajectory linearization: https://underactuated.mit.edu/lqr.html
- acados features, RTI phases and partial condensing: https://docs.acados.org/features/
- OCS2 methods and switched-system interfaces: https://leggedrobotics.github.io/ocs2/overview.html
- OSQP convex MPC formulation: https://osqp.org/docs/examples/mpc.html

The reusable boundary is model, derivatives, constraints, estimator information, control reference, numerical diagnostics and execution timing. Reusing that boundary does not authorize copying CartPole parameter values or safety claims to another robot.
