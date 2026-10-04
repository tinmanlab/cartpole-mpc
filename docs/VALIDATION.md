# Validation

Runtime/asset execution authority: [MuJoCo WASM runtime](MUJOCO_WASM_RUNTIME.md). Browser and commissioning use the official WASM engine; legacy equation tests are explicitly differential references.
## Reproduce

    npm ci
    npm run check

The check runs:
- JavaScript syntax validation,
- standalone index rebuild parity,
- deterministic engine probes,
- mathematical regression checks for controller/observer internals.

For the independent SciPy/NumPy cross-check:

    python3 scripts/audit_reference.py

For the browser/WebMCP smoke test:

    python3 tests/test_browser.py

## Algorithm audit

The audited implementation checks:
- LQR gain against SciPy solve_discrete_are,
- observability rank of the [position, angle] sensor model,
- one complete KF predict/update against independent NumPy equations,
- Linear MPC adjoint gradient against finite differences,
- Linear MPC fixed-point solution against an independent bounded SciPy optimization,
- Full NMPC fixed-point solution against an independent nonlinear bounded SciPy optimization,
- reduced CoM acceleration against net horizontal force / total mass,
- Joseph-form covariance symmetry / positive-semidefinite behavior,
- timestamp alignment through an exact-zero Truth-observer RMSE regression,
- the authority/covariance boundary of the learned residual observer.

The regenerated receipt is:

    evidence/algorithm_audit.json

## Current fixed probes

The checked-in tests also verify:
- PID, LQR, Linear MPC, Centroidal-style MPC, Full NMPC, and PPO survive the nominal truth-state probe,
- Linear MPC exposes N=30,
- Centroidal-style MPC exposes N=32 and a downstream reference,
- Full NMPC exposes N=30 and a bounded iterative solve,
- Full NMPC reports finite solver timing, but wall-clock timing is host-dependent and is not a portable CI acceptance gate,
- raw finite-difference estimation fails in most seeds under the fixed strong sensor-noise probe,
- the estimator nonlinear bench fixes LQR, starts at 0.5 rad with very low sensor noise and no push, and yields KF RMSE about 0.0080 versus EKF about 0.00323 across the fixed five-seed check,
- the SO(2) wrap regression maps +179 deg to -179 deg to a +2 deg residual instead of a -358 deg Euclidean jump,
- Adaptive-R down-weights the deterministic measurement-glitch probe relative to the fixed-R KF.

More exhaustive generated measurements are stored in:

    evidence/control_observer_v2_metrics.json

Those numbers are paired CartPole evidence only. They are not an algorithm ranking or humanoid benchmark.

## Demo recording

The README animation is generated from a real browser run:

    python3 scripts/record_demo.py

The audited front-page recording uses:
- Full nonlinear NMPC,
- EKF,
- mixed sensor-noise + model-mismatch scenario,
- external pushes and goal changes.

The recording is an interface demonstration, not a benchmark.

## Frozen terminal-cost validation

`tests/fixtures/terminal_validation.json` is the prospective manifest fixed before the first verification pass. SHA-256: `23594cd723f706ddc881bc84717b5d28cc17370508502a11debcd7662b847742`. It contains 24 primary paired conditions (24 seconds each) and 16 boundary paired conditions (30 seconds each), including unseen seeds, initial states, reference reversals and fixed external pushes. No conditions, weights or acceptance tolerances were tuned after inspecting these cases.

The committed `scripts/validate_terminal.mjs` now invokes the public `{terminalCost: 'dare'}` constructor option instead of assigning a test-only Qf. Every plant step and estimator prediction uses actual MuJoCo WASM. The original and candidate use the same sensor/randomization setup; hidden plant parameters are not controller inputs. Simulator-derived R remains labelled `injected-noise-oracle`.

`tests/test_terminal.mjs` covers full cross terms, changed Q/R, unchanged defaults, finite inputs, unsupported/conflicting options, forced nonconvergence, positive cost checks, rail coordinates and runtime metadata. `tests/test_terminal_reference.py` independently compares four actual-WASM model/cost combinations with SciPy DARE and OSQP. Limits: P max-absolute error 1e-6, normalized DARE residual 1e-9, first-input error 0.001 N and relative QP cost error 1e-4. Existing QP acceptance tolerances are unchanged.

`evidence/terminal_validation.json` reports every trial, applied steps, failure category, state/goal, tracking and effort, solver residuals and host timing. Full-duration paired MSE uses only cases completed by both designs and reports exclusions. Common-prefix metrics are retained for early exits. Numerical guard rejection remains a failed trial, distinct from solver infeasibility and plant-envelope failure. A valid experiment is not automatically a passed performance screen; the two fields are separate. The empirical screen is not a general safety or automatic default-promotion decision.

The browser regression checks selector state, actual terminal matrix, trace/probe/comparison propagation, unsupported choices, reset persistence, and constructor failure stopping without advancing the old plant. One page explicitly tests WebMCP callbacks using a registration shim; the second tests the normal UI without a shim. No native WebMCP support is fabricated.

Reproduce using the existing native reference environment:

```sh
npm run check
.venv-native/bin/python tests/test_terminal_reference.py
npm run validate:terminal
npm run test:browser
```

The existing verify workflow runs these checks; there is no separate service or new workflow framework. Earlier commissioning receipts remain historical at their recorded source identities. This option does not imply that the 13-section commissioning campaign was regenerated or that hardware was evaluated.

## Educational correctness regression

`tests/test_education.mjs` checks all lesson scopes/sources, real five-model count, unavailable convergence for local-update modes, genuine reduced-only prediction and componentwise metrics. A pre-edit local control-sequence capture also checks that reporting corrections preserve sampled actions. Browser regression visits all 28 topics, verifies source links and readable line breaks, checks that reading leaves runtime unchanged, and inspects reduced-forecast/metric-unit metadata.

A scripted string/metadata test is not proof of every explanatory sentence. Primary-source review, code inspection, numerical checks and human-readable UI inspection remain separate evidence. Earlier dated audit/commissioning files retain their source snapshots; do not cite them as freshly rerun campaign results.
