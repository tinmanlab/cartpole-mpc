# Constraint-active CartPole diagnostic

This small follow-up addresses review F2 without changing the previous tuning experiment. The five cases were committed before the runner was implemented at `349be01c2042df14b32764db3c72dd003c37c4e1`. The recorded protocol, not an intended winner, determines the experiment. Later reuse is regression evidence.

## What changes and what stays fixed

`tests/fixtures/constraint_diagnostic.json` specifies an interior case, mirrored right/left rail-approach cases, and their model-error counterparts. The near-rail cart starts 0.05 m from the rail, moving outward at 0.3 m/s, with 0.12 rad pole tilt. These conditions deliberately probe a harder operating region; they are not a random sample establishing reliability or a global ranking.

All four original pairs run: LQR/KF, LQR/EKF, hard-rail MPC/KF, hard-rail MPC/EKF. Retain r1-q1, measured Re, Qe/P0, horizon 30, force +/-10 N, world rail +/-2.4 m and the original 600-step task/evaluation. No parameter search, new solver, extra motor or new dynamics model is added. Existing source and historical tuning results are unchanged.

## Two different questions

**Exact-state plan:** On the existing nominal linear model, compare the clipped LQR prediction and constrained MPC plan from the exact initial state. Report the complete planned inputs/states, nominal rail activity, cost and residuals. This is a model-only diagnostic. It is not a physical run and does not mean the controller has access to true velocity in the sensor-based experiment.

**Sensor-based physical run:** The existing DesignStudy runner uses real MuJoCo WASM, noisy position/angle measurements, and KF/EKF outputs. As in that runner, estimated velocities initialize at zero: the real nonzero initial velocity is not provided. The executed controllers retain their original initialization; no warm-up is executed. Near a 0.05 m margin, 0.06 m position measurement noise and initialization errors can dominate. A nominal feasible plan therefore does not establish a usable noisy recovery domain.

Record all solver rejections, applied steps, actual rail/angle failures, final-task results, and predicted-rail-active sample counts. No full-duration score is assigned to a partial run. The true-state one-step residual is evaluator-only and combines linearization, model and commanded/applied-input mismatch; it is not called an observer error.

## Independent numerical verification

`tests/test_constraint_diagnostic.py` executes the runner and checks its information, model and outcome contracts. The original independent `solve_active_rail` OSQP implementation handles the same nonzero reference using one algebraic constant coordinate:

    z = [world_state, 1]
    error = [I, -target] z

This rewrites an affine linear prediction into homogeneous coordinates and preserves world-position constraints. It is not another physical plant, estimator state or new solver. Complete original costs, accepted physical constraints and first inputs are independently compared. Missing jointly admitted solutions are reported as unavailable comparison, not zero numerical error or mathematical impossibility.

The test does NOT demand that MPC wins, all physical trials finish, or a minimum number of plans become active. A valid negative result remains a useful diagnostic. Claim only the rail activity and physical outcomes actually observed; do not modify the cases after observing them to force a preferred answer.

## Run and inspect

From the existing Node and native-reference environment:

    python tests/test_constraint_diagnostic.py
    python3 scripts/constraint_diagnostic_report.py

The runner writes detailed traces to `test-results/constraint_diagnostic_full.json`; independent verification writes `evidence/constraint_diagnostic_summary.json` and the source-bound generated `evidence/constraint_diagnostic_report.md`, and prints a compact `CONSTRAINT_DIAGNOSTIC_RESULT` in the CI log. The ordinary repository verification suite remains in place. This patch does not add an online lesson screen or publish Pages.

The resulting distinction is intentionally narrow: optimizer acceptance, nominal constraint satisfaction, estimator initialization, true model behavior, and task success are separate evidence. The experiment neither retunes the existing recommendation nor establishes general LQR/MPC superiority, stochastic safety, or hardware authority.

## Reporting correction and offline initialization ablation

The original CI receipt and fresh base reproduction agree on **4/20 task passes and 16 MPC rail-active samples**. The earlier 6/20 and 44-sample narrative was incorrect. Historical evidence and the frozen protocol remain unchanged. The table is now generated from the validated machine receipt; content integrity, current source hashes and recomputed outcome counts are checked before rendering. This detects stale or accidentally altered receipts, not maliciously re-authored evidence.

The runner retains the initial state and the first rejected attempt **before any attempted command is applied**, at the same physical timestamp: current measured position/angle and freshness, estimate, covariance, evaluator-only truth, applied-step count, goal/design identity and rejection reason. It never draws another measurement. Each KF/EKF is separately replayed from recorded causal measurements and commands and must reproduce the same estimate and covariance. A separately instantiated cold reset uses only the snapshot's current measurement and original P0. This is an offline initialization ablation, not an executed controller or measured successful warm-up. No true velocities enter either observer.

At these MPC snapshots only, the unchanged affine OSQP reference compares the same nominal QP for continuous-history, cold-reset and evaluator-only true states. Step-zero duplicates are combined. Initial-state-outside, solver-declared infeasible, numerical nonacceptance/iteration limit and acceptance are distinct categories. Solver rejection is not a mathematical infeasibility certificate or evidence of plant impossibility. Joint admission permits comparison of the first action and original cost; unavailable comparisons stay unavailable.

Position mean +/- 2 sqrt(Ppp) is an **assumed-covariance diagnostic**, accompanied by evaluator-only error and same-time covariance identity. It is not calibrated risk, a robust margin, safety permission or simultaneous confidence coverage. Zero executed transitions have a null one-step residual and an explicit zero sample count; genuine zero residuals with samples remain zero. Raw traces stay in `test-results/`; only the compact receipt and generated report are retained. No recommendation or default changes follow from this ablation.
