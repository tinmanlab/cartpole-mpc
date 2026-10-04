# Constraint-active CartPole diagnostic

This small follow-up addresses review F2 without changing the previous tuning experiment. The five cases were committed before the runner was implemented at `349be01c2042df14b32764db3c72dd003c37c4e1`. The recorded protocol, not an intended winner, determines the experiment. Later reuse is regression evidence.

## What changes and what stays fixed

`tests/fixtures/constraint_diagnostic.json` specifies an interior case, mirrored right/left rail-approach cases, and their model-error counterparts. The near-rail cart starts 0.05 m from the rail, moving outward at 0.3 m/s, with 0.12 rad pole tilt. These conditions deliberately probe a harder operating region; they are not a random sample establishing reliability or a global ranking.

All four original pairs run: LQR/KF, LQR/EKF, hard-rail MPC/KF, hard-rail MPC/EKF. Retain r1-q1, measured Re, Qe/P0, horizon 30, force +/-10 N, world rail +/-2.4 m and the original 600-step task/evaluation. No parameter search, new solver, extra motor or new dynamics model is added. Existing source and historical tuning results are unchanged.

## Two different questions

**Exact-state plan:** On the existing nominal linear model, compare the clipped LQR prediction and constrained MPC plan from the exact initial state. Report the complete planned inputs/states, nominal rail activity, cost and residuals. This is a model-only diagnostic. It is not a physical run and does not mean the controller has access to true velocity in the sensor-based experiment.

**Sensor-based physical run:** The existing DesignStudy runner uses real MuJoCo WASM, noisy position/angle measurements, and KF/EKF outputs. As in that runner, estimated velocities initialize at zero: the real nonzero initial velocity is not provided. There is no warm-up measurement history in this diagnostic. Near a 0.05 m margin, 0.06 m position measurement noise and initialization errors can dominate. A nominal feasible plan therefore does not establish a usable noisy recovery domain.

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

The runner writes detailed traces to `test-results/constraint_diagnostic_full.json`; independent verification writes `evidence/constraint_diagnostic_summary.json` and prints a compact `CONSTRAINT_DIAGNOSTIC_RESULT` in the CI log. The ordinary repository verification suite remains in place. This patch does not add an online lesson screen or publish Pages.

The resulting distinction is intentionally narrow: optimizer acceptance, nominal constraint satisfaction, estimator initialization, true model behavior, and task success are separate evidence. The experiment neither retunes the existing recommendation nor establishes general LQR/MPC superiority, stochastic safety, or hardware authority.
