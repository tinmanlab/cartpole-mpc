# Model hierarchy: fixed linearization, LTV, NMPC, reduced order, and hybrid systems

A common source of confusion is the phrase "use the nonlinear model directly."

There are several distinct levels.

## 1. Fixed LTI model

Linearize once around an equilibrium:

    delta x_(k+1) = A delta x_k + B delta u_k

The same A and B are reused at every time.

Advantages are speed, convex formulations, and easy analysis. The limitation is that validity is local to the operating region.

This is what the CartPole LQR/KF/Linear MPC use around upright.

## 2. LTV / successive-linearization MPC

Keep a nonlinear model as the source of truth, but relinearize along the current nominal trajectory:

    A_k = df/dx at (xbar_k, ubar_k)
    B_k = df/du at (xbar_k, ubar_k)

Then solve one local time-varying subproblem.

The LTV MPC / SQP-RTI bridge performs one real-time-iteration-style update per control tick:

    warm start
    → nonlinear nominal rollout
    → trajectory-dependent Jacobians
    → one local quadratic update
    → apply first command
    → repeat next tick

It is not an acados or OCS2 source port.

## 3. Nonlinear MPC

The OCP itself retains:

    x_(k+1) = f(x_k, u_k)

The solver still usually uses derivatives and local approximations internally.

Therefore NMPC does not mean "never linearize." It means the nonlinear dynamics remain part of the optimization problem rather than being replaced permanently by one fixed A,B model.

The CartPole Full NMPC uses nonlinear rollout and multiple local iLQR-style iterations.

## 4. Reduced-order MPC

For large robots, the full rigid-body system may be too expensive or unnecessarily detailed for the planning layer.

Centroidal MPC retains quantities such as center of mass, centroidal momentum, contact wrench, and selected configuration variables. A lower-level inverse-dynamics/WBC layer realizes the reduced plan.

The CartPole centroidal-style controller is a structural analogue:
- reduced horizontal CoM prediction,
- future CoM reference,
- full-state downstream stabilization.

## 5. Full-order rigid-body NMPC

A humanoid full-order OCP can include floating-base configuration, joint configuration, velocities, contact wrench, joint acceleration or torque, and collision/contact constraints.

This is qualitatively different in scale from four-state CartPole.

Use sparse rigid-body dynamics and mature solvers rather than scaling the browser implementation.

Recommended native authorities for Figure-like work:
- Pinocchio for rigid-body dynamics and derivatives,
- OCS2 for switched/nonlinear optimal control,
- acados for NMPC/MHE, multiple shooting, SQP/RTI and sensitivities.

## 6. Humanoids are hybrid systems

Humanoid dynamics change with contact mode:

    left support
    double support
    right support
    flight

Constraints can include normal force >= 0, friction cone, stance-foot velocity = 0, swing clearance, joint/torque/velocity limits, and self collision.

So "nonlinear" is still not the whole story. A transferable architecture must represent continuous nonlinear dynamics, discrete contact modes, path constraints, mode-dependent constraints, and contact uncertainty.

## 7. Estimation has the same hierarchy

    KF     fixed linear dynamics
    EKF    nonlinear mean + local Jacobian
    InEKF  invariant error when Lie-group structure is useful
    MHE    optimize a recent window with dynamics/measurements/constraints
    NMHE   nonlinear moving-horizon problem

The CartPole Nonlinear shooting MHE is intentionally limited:
- EKF posterior is used as an approximate arrival prior,
- only the first state of the window is optimized,
- the nonlinear plant is single-shot through the window,
- no process-noise trajectory or hard state constraints are decision variables.

A full humanoid MHE should use a mature sparse QP/NLP implementation. A practical legged example combines orientation EKF with constrained velocity MHE at 200 Hz:
https://arxiv.org/abs/2405.20567

## Transfer rule

The transferable idea is not a particular solver.

The transferable contract is:

    model(x,u,mode,parameters)
    derivatives(x,u)
    constraints(x,u,mode)
    estimator(sensor history, controls)
    controller(estimated state, reference)
    diagnostics(residuals, constraint margins, timing)
    uncertainty/model parameters
    scenario generator

Figure should map this contract to its real rigid-body/contact stack rather than reusing CartPole-specific equations.
