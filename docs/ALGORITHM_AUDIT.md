# Algorithm Correctness Audit — 2026-10-03

Base public revision audited: `0b17aa572d432c33614fd02db26a3d2a5f9ba519`.

This audit treats four things separately:

1. mathematical correctness of the algorithm,
2. fidelity between the algorithm and the shared CartPole plant,
3. correctness of the controller/observer evaluation contract,
4. correctness of the educational claims.

## Findings

### F1 — HIGH / blocking: estimator and truth were compared at different timestamps

Public-main loop order was effectively:

    y_k -> observer predict/update -> xhat_(k+1-ish)
    -> controller -> plant -> x_(k+1)

and then the trace compared the pre-plant estimate with the post-plant truth.

That makes observer RMSE and controller-observer comparisons conceptually invalid.

Correction:

    xhat_k -> controller -> u_k
    -> plant -> x_(k+1)
    -> sensor -> y_(k+1)
    -> observer predict/update with u_k
    -> xhat_(k+1)

Regression: Truth/oracle observer RMSE must be exactly zero. Current audited result: 0.

### F2 — HIGH / blocking: KF/EKF measurement R did not follow the selected sensor-noise scenario

The plant changes injected measurement standard deviations between nominal, sensor, model, mixed, and glitch scenarios. Public main left the KF-family R at the nominal default.

That made a sensor-noise experiment confound:
- more noise in the plant,
- an estimator that was incorrectly told the old noise level.

Correction:

    R = diag(sigma_position^2, sigma_angle^2)

for each scenario's Gaussian sensor model.

The deterministic glitch is deliberately excluded from baseline R so it remains an outlier test.

### F3 — MEDIUM: runtime linear model did not exactly match the discrete plant being simulated

Public main used a hand-derived continuous linearization and forward Euler over 20 ms, while the live plant uses four 5 ms semi-implicit steps.

The approximation is reasonable near upright, but it is an avoidable teaching mismatch.

Correction: runtime LQR, KF, and Linear MPC use the numerical discrete Jacobian of the same 20 ms nonlinear transition at the upright equilibrium. The continuous Ac/Bc remain for derivation/explanation.

### F4 — MEDIUM: Linear MPC name overstated the original solver

Public main computed finite-horizon unconstrained feedback gains and clipped each resulting force. That is useful saturated finite-horizon LQR behavior, but it is not the same as solving the box-constrained horizon optimum.

Correction:
- warm-started control sequence,
- analytic adjoint gradient,
- projection onto `|u_k| <= u_max`,
- backtracking line search,
- receding first-control application.

Independent SciPy bounded-optimization audit at the fixed point:

    JS cost              20.0284372646
    SciPy optimum cost   20.0283641344
    relative cost gap    3.65e-6
    first-action error   0.0151 N

Gradient vs finite differences: max absolute error about 4.37e-10.

### F5 — MEDIUM: learned residual output and covariance scope were visually conflated

The residual correction is an output augmentation around the base EKF. The base filter covariance does not automatically become the covariance of the neural-corrected estimate.

Correction:
- corrected state is output-only,
- base EKF continues independently,
- `baseP` remains available diagnostically,
- corrected-output `P` is reported unavailable,
- UI does not draw base-P confidence as corrected-output confidence.

This matches the authority boundary taught for InNKF.

### F6 — MEDIUM: CoCo-InEKF and Adaptive-R were conflated

Public wording grouped the heuristic Adaptive-R demo under “CoCo/FOCUS”.

That is too broad.

CoCo-InEKF learns continuous contact-candidate velocity/process covariances for persistent contact candidates inside a differentiable InEKF. It is not simply an observation-R inflation heuristic.

Correction:
- CoCo page is conceptual only in CartPole,
- Adaptive-R is labeled an outlier/observation-reliability bridge,
- the executable bridge is associated only with the mathematical idea of down-weighting unreliable observations.

### F7 — LOW/MEDIUM: EKF -> InEKF was easy to read as a universal upgrade ladder

Correction: InEKF is presented as a branch that becomes appropriate when the state/dynamics admit useful Lie-group / group-affine structure. The CartPole runtime exposes only the SO(2) wrapped-angle error issue and is not called a Hartley InEKF implementation.

### F8 — LOW: solver-iteration and wall-clock diagnostics overstated semantics

A zero projected-gradient update could be displayed/count as one iteration due to `|| 1`. Runtime solve-time was also used as a portable test gate despite being host dependent.

Correction:
- report actual iteration count, including zero,
- keep solver milliseconds as diagnostics,
- do not use host wall-clock timing as a portable algorithm-correctness CI gate.

## Independent numerical checks

Regenerate with:

    npm run check
    python3 scripts/audit_reference.py
    python3 tests/test_browser.py

Current independent receipt: `evidence/algorithm_audit.json`.

### LQR / DARE

Compared against SciPy `solve_discrete_are`:

    max |K_js - K_scipy| = 1.86e-7

All audited closed-loop eigenvalues are inside the discrete unit circle.

### Controllability and observability

For state `[p, v, theta, omega]`, horizontal force input, and direct measurement `[p, theta]`:

    rank(C) = 4
    rank(O) = 4
    state dimension = 4

The local upright linearized system is controllable and observable.

### Kalman Filter

One complete predict/update was recomputed independently in NumPy.

    state max error       6.94e-18
    covariance max error  8.13e-20
    gain max error        3.33e-16
    innovation error      0

Joseph-form covariance update is used in runtime.

### Full nonlinear NMPC

A fixed horizon problem was independently solved with SciPy bounded nonlinear optimization.

    JS cost              27.3020168999
    SciPy optimum cost   27.3020169009
    first-action error   5.59e-6 N

The current implementation is accurately described as a small single-shooting iLQR-style NMPC, not as OCS2 multiple-shooting SQP.

### Reduced CoM model

For the ideal CartPole horizontal external-force relation:

    finite-difference c_ddot   1.8181798351
    F / total_mass             1.8181818182
    absolute error             1.98e-6

This validates the reduced horizontal CoM relation used by the centroidal-style planner.

## Educational-use rules after audit

Use the lab to learn structural differences, not to rank algorithms.

Good comparisons:
- Raw vs KF under sensor noise.
- KF vs EKF as nonlinear excursion/model mismatch increases.
- EKF vs SO(2) bridge around angle wrapping.
- Fixed-R KF vs Adaptive-R under the deterministic measurement glitch.
- LQR vs Linear MPC to isolate horizon/constraint effects.
- Linear MPC vs Centroidal-style vs Full NMPC to isolate model reduction versus nonlinear horizon prediction.

Do not infer:
- PPO is worse/better from the common lab table; its original training contract differs.
- InEKF is the automatic successor to EKF.
- Adaptive-R is CoCo or FOCUS.
- lower CartPole RMSE proves a humanoid estimator is superior.
- browser solve milliseconds imply a hardware real-time guarantee.
