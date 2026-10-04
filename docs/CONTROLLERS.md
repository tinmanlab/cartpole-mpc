# Controllers

This page distinguishes controller families by what mathematical structure they add. It is not a ranking.

## Shared plant and timing

All controller modes command the same nonlinear CartPole plant.

- control period: 20 ms (50 Hz)
- plant integration: four 5 ms official MuJoCo WASM Euler steps per control period
- input: horizontal force, clipped to the configured actuator limit
- local failure boundary: cart track and a deliberately wider pole-angle envelope than the original PPO training task

For LQR, KF, and linear MPC, the runtime discrete A and B matrices are numerical Jacobians of this same 20 ms nonlinear transition at the upright equilibrium. The hand-derived continuous Ac and Bc remain in the source for explanation, but are not silently forward-Euler-discretized for the runtime controller.

## PID

Purpose: direct feedback without an explicit plant model.

    e_p = position - target
    u = k_theta theta + k_omega omega
        + k_p e_p + k_v velocity
        + k_i integral(e_p)

The position integral is clamped to limit wind-up in this toy implementation.

## LQR

Purpose: derive a state-feedback gain from a linear model and quadratic cost.

    x_(k+1) = A x_k + B u_k
    J = sum(x^T Q x + u^T R u)
    u = -Kx

The code solves the discrete algebraic Riccati equation iteratively. The algorithm audit cross-checks K against SciPy solve_discrete_are; the current maximum absolute difference is below 2e-7.

LQR's Riccati problem has no input constraint. The implementation saturates the resulting feedback force to the actuator limit before sending it to the plant; that saturation is outside the LQR optimum.

## Linear MPC

Purpose: add a finite horizon and an explicit input box constraint.

    minimize  sum x_k^T Q x_k + R u_k^2 + x_N^T Q_f x_N
    subject to
              x_(k+1) = A x_k + B u_k
              |u_k| <= u_max

The current controller uses N=30 and the pinned upstream `quadprog@1.6.1` Goldfarb–Idnani solver. The local adapter constructs the finite-horizon QP and checks finite values, primal feasibility, dual sign, complementarity and stationarity before applying the first control. The solver does not use the previous control sequence as a warm start; retaining a shifted sequence in the controller is not evidence of solver warm starting.

The `hard_mpc` variant adds world-coordinate rail bounds at every predicted stage. The `state_mpc` comparison remains a soft-penalty method. Their constraint semantics are different, and neither constitutes a robust safety guarantee for the true mismatched plant. Independent OSQP comparisons cover the unchanged four input-box cases and an active-rail case. See `MUJOCO_WASM_RUNTIME.md` for runtime and evidence boundaries.


## Scenario-risk MPC

Purpose: make model uncertainty part of the optimization rather than only part of an external stress test.

The CartPole controller builds three local linear models with different cart mass, pole mass and pole length. One shared control sequence is evaluated on all of them. The objective is

    J = mean_i J_i + rho * (max_i J_i - mean_i J_i)

with the same input box constraint for every scenario. The gradient combines the mean gradient with the gradient of the current worst model.

This is an executable finite-scenario risk controller. It teaches the distinction between:

    nominal MPC
    scenario/risk robustification
    domain-randomized validation

It does **not** provide the invariant-set guarantee of tube MPC, the adversarial guarantee of a formal min-max controller, or a chance-constraint probability guarantee. For humanoids, select the robustness formulation according to measured uncertainty and use the native OCP solver.

## Centroidal-style MPC

The humanoid architecture that motivated this page uses a reduced state based on centroidal momentum/configuration, contact-wrench inputs, and a lower-level realization layer.

CartPole has no feet or independent contact-wrench decision variables. The implementation therefore preserves the hierarchy without inventing fake contacts.

For the uniform pole model used by the plant, with l as pivot-to-pole-CoM distance:

    M = m_cart + m_pole
    c = x + (m_pole * l / M) sin(theta)
    c_dot = v + (m_pole * l / M) cos(theta) omega

Under the ideal horizontal reduced model:

    c_ddot = F_net / M

The outer planner uses a 32-step box-constrained MPC on [c, c_dot]. A look-ahead CoM reference is then passed to a full-state LQR stabilizer.

    reduced CoM MPC
          ↓
    future [c, c_dot] reference
          ↓
    full-state stabilization
          ↓
    physical force

External pushes and model mismatch are intentionally unknown to the outer reduced model.

This is an actual reduced-order controller and a structural analogue of the humanoid centroidal-planner/lower-level-control hierarchy. It is not an OCS2 centroidal source port.

## Full nonlinear NMPC

This controller puts the same nominal nonlinear CartPole transition used by the live simulator inside its prediction horizon.

At each control tick:
1. warm-start the previous force sequence,
2. nonlinear rollout over N = 30,
3. compute central-difference stage Jacobians A_k = df/dx and B_k = df/du,
4. form an iLQR-style local quadratic approximation,
5. run a backward Riccati-like solve,
6. run a bounded forward line search with input clipping,
7. repeat at most twice,
8. apply only u_0,
9. shift the sequence.

This is a genuine nonlinear receding-horizon controller, more specifically a small single-shooting iLQR-style NMPC implementation. Here "full" means the full four-state CartPole model is optimized rather than a reduced CoM model; it does not mean the humanoid full-order state from wb_humanoid_mpc is reproduced.

It does not implement:
- OCS2 sparse multiple shooting,
- general nonlinear state constraints,
- exact active-set handling of input constraints,
- globalization or convergence guarantees equivalent to a production SQP solver.

The force bound is enforced on candidate controls during the forward pass.

## PPO

The PPO mode uses the frozen discrete actor from the related CartPole PPO Studio.

Runtime evaluation is deterministic: the larger of the two action probabilities selects -force or +force.

Important comparison boundary:
- the actor was trained under the PPO Studio task contract,
- this lab uses its own local failure envelope and can route arbitrary observer estimates into the actor,
- therefore the controller × observer table is an educational common-plant comparison, not the original PPO benchmark or a retraining result.

The conceptual contrast is:

    MPC/NMPC: solve a model-based optimization online
    PPO:      move the main optimization into training, then infer online

Neither is presented as universally superior.

## Terminal cost selection

`LinearMPCController` and `hard_mpc` accept `{terminalCost: 'original' | 'dare'}`. Both the constructor and browser initially use `original`, preserving the previous diagonal penalty and existing experiment baselines. The browser's Terminal cost selector opts into `Riccati · full matrix`; changing it resets the experiment, and Reset retains that selection. Reload starts with Original. Other controller families disable the selector and reject explicitly supplied unsupported terminal options rather than silently applying them.

The Riccati choice uses the actual nominal discrete A/B and the controller's Q/R. It reuses the existing DARE iteration once during construction, retaining all matrix cross terms. The matrix is checked for finite values, convergence, round-off-scale symmetry, strict positive definiteness without jitter, and a normalized infinity-norm Riccati residual no greater than 1e-9. This restricted mode requires positive-definite state cost and positive scalar input cost. A supplied `Qfdiag` conflicts with `terminalCost: 'dare'` and is rejected. Original still accepts its configured diagonal.

Runtime `terminalCost` metadata distinguishes computed nominal Riccati from configured diagonal cost. The browser state also includes the actual terminal matrix; each trace row records its terminal kind. Comparison rows capture the terminal choice at the start of the comparison and label it. The existing `cartpole_set_controller` and `cartpole_run_probe` actions accept an optional terminalCost field; an omitted probe field follows the current selection only for the two supported families.

A failed constructor leaves the previous complete state snapshot stopped, not a new plant running with an old controller. It requires a successful reset/reconfiguration. The startup path still fails visibly if there is no valid initial configuration.

This is a local nominal tail-cost approximation, not a terminal invariant set, global nonlinear stability proof, robust recovery region or hardware certificate. Physics remains official MuJoCo WASM and the browser QP solver remains the pinned upstream quadprog. No acados runtime or new dependency is added by this option.
