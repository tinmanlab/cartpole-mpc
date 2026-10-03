# Controllers

## PID

Purpose: direct feedback without an explicit plant model.

    u = k_theta theta + k_omega omega
        + k_p position_error + k_v velocity
        + k_i integral(position_error)

Use it to learn what feedback alone can do before introducing optimal control.

## LQR

Purpose: derive one state-feedback gain from a linear model and quadratic cost.

    x_(k+1) = A x_k + B u_k
    J = sum(x^T Q x + u^T R u)
    u = -Kx

The Riccati recursion used here is also important because related quadratic subproblems appear inside MPC/NMPC.

## Linear MPC

Purpose: make the future horizon explicit.

The controller uses the upright linear model over N=30, computes a finite-horizon feedback sequence, applies the force limit, uses the first control, then resolves next tick.

The horizon is real. It is not a decorative predicted line.

## Centroidal-style MPC

The humanoid architecture that motivated this page uses a reduced state based on centroidal momentum, configuration, contact wrench inputs, and a lower-level realization layer.

CartPole cannot reproduce feet/contact wrenches honestly. This implementation therefore uses the real whole-system horizontal CoM:

    M = m_cart + m_pole
    c = x + (m_pole * l / M) sin(theta)
    c_dot = v + (m_pole * l / M) cos(theta) omega

A reduced predictive controller plans future CoM motion. A full-state stabilizer then realizes that reference on the underactuated CartPole.

This preserves the important hierarchy:

    reduced predictive planner
             ↓
       future reference
             ↓
    lower-level stabilization
             ↓
          actuator

It is an architecture analogue, not an OCS2 source port.

## Full nonlinear NMPC

This controller puts the same nonlinear CartPole transition used by the live plant inside its horizon.

Each control tick:
1. warm-start the previous force sequence,
2. nonlinear rollout over N=30,
3. compute numerical stage Jacobians A_k and B_k,
4. run a local quadratic backward solve,
5. bounded forward line search,
6. repeat at most twice,
7. apply only u_0,
8. shift the sequence.

This is an actual nonlinear receding-horizon controller.

It is not claimed to reproduce OCS2 sparse multiple-shooting SQP.

## PPO

The PPO mode uses a frozen actor from the related CartPole PPO lab.

The comparison is conceptual:

    MPC/NMPC: solve an optimization problem online
    PPO:      move optimization into offline training, then infer online

Neither is presented as a universal winner.
