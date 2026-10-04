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

## Measurement-calibration causal lesson

This slice completes one explicit learning question on the **single pendulum**: how an assumed measurement covariance affects state correction, force requests, saturation and target tracking. It does not increase link count or move to a legged-robot implementation. It reuses `LabPlant`, the actual MuJoCo WASM backend, `EKFObserver`, `LQRController` and the existing matrix operations. No new solver, estimator algorithm, optimization service or dependency is introduced.

### Scope and frozen experiment

`tests/fixtures/calibration_lab.json` was written before running these cases; SHA-256 `6300111aae39c0858702d430de7ec04cb03d926be923858bf6d0e85412188698`. There is one stationary acquisition (seed 6201, 512 position/angle samples), four independent-white-noise dynamic cases, two correlated-noise cases and two command-delay cases. Dynamic seeds differ from the acquisition seed. Each case runs 600 steps = 12 s using three R_e choices: the explicit nominal small-noise assumption, the measured diagonal covariance, and 100 times that measured covariance. The last arm is deliberately wrong as a measurement-variance model, not an optimized or recommended calibration.

Q_e, LQR Q_c/R_c/gain, force limit, physical model, initial conditions, target and scheduled pushes remain fixed between arms. Test-fixture sensor amplitudes are held at 0.06 m and 0.025 rad for all dynamic cases. Only the boundary mechanism (temporal correlation or existing two-tick command delay) differs. This fixes an interpretable intervention rather than simultaneously changing gains, physics and estimator type. No performance result selects or promotes a new default; failures and regressions are retained.

### Data-only calibration versus simulator knowledge

The fit function receives only two-channel readings and a declared diagnostic-screen configuration. It does NOT read simulator truth, `measurementVariance()`, injected sensor sigma, controller outcomes or validation data. Tests replace the injected-variance API with a throwing function and run the entire fitting/filter path. Adding a constant offset to all samples leaves the estimated covariance unchanged; this also demonstrates that the mean cannot identify absolute sensor bias without an external reference.

The sample covariance uses the ordinary centered n−1 estimator. The current two-channel estimator consumes diagonal R, so measured channel correlation and lag-one correlation are exposed. Excessive correlation makes the proposed diagonal independent-noise fit unavailable under the declared heuristic screen rather than silently declaring success. Passing this screen does not prove stationarity, independence or Gaussianity. Angle samples are handled in a local circular chart; a wide incompatible angular record is rejected.

The calibration acquisition is explicitly a **clamped stationary fixture** whose sensor is repeatedly read, not a freely falling pole held upright by an unreported controller. The simulator creates those readings; this is not a hardware dataset. A real equivalent would require a genuinely stationary fixture and appropriate acquisition protocol. The fit estimates short-term repeatability around an unknown constant, not process Q_e, mean bias, kinematic offsets, actuator dynamics or delay. Full covariance and covariance provenance are retained even though only its diagonal is passed to the current EKF.

The existing `src/commissioning.js` CEM calibration is not replaced or relabelled as this method. It remains an offline broader heuristic study, including its own information privileges and selection limits. The new small shared lesson function is needed to expose a sensor-data-only R experiment in both Node and the browser without importing the whole commissioning search.

### Honest causal timing and truth use

Each row carries measurement/estimate/truth at t_k, the force generated at t_k and its application over [t_k,t_(k+1)). The next measurement is used only at the next row. Initial covariance and measurement initialization do not invent an initial prefit innovation/NIS.

For the fixed linear gain, the evaluation-only identity is:

    u_requested(x_hat) = u_requested(truth) - K (x_hat - truth)

The view splits the last term into the four state contributions with explicit units. The actual controller receives x_hat only; the truth-based force is never applied or used to choose a gain. Requested force, clipped command, actuator-applied force and external push are distinct. The reference test reconstructs these relationships, covariances and NIS independently using NumPy. A clipped/nonlinear controller generally cannot have its entire policy error represented by this linear identity; the present decomposition is explicitly for the LQR request before clipping.

Paired closed loops have the same RNG/fixture but not the same measurements after different control actions produce different true trajectories. Therefore the lesson also replays all three filters on one fixed measured-R trajectory's sensor/command record without advancing the plant or applying new control. The recorded arm reproduces its own estimates numerically. Replay estimation errors cannot be mistaken for a new controller's closed-loop tracking performance.

### Observations, not a universal winner

In the initial four white-noise conditions, all arms completed 600 steps, but not every run met the separately defined final-window task criterion. The explicit small-R assumption accumulated 79 saturated samples, measured R accumulated 2, and 100×R accumulated 0, out of 2400 samples per arm. Equal-duration position-tracking RMSE was approximately 0.139, 0.133 and 0.152 m respectively. Individual cases include worse tracking after calibration despite smaller state-estimation error/force variation.

The oversized R can produce a smaller position-estimation RMSE in some cases because the rest of the filter/model (including assumed Q_e) is not jointly calibrated here. A measured R correctly describes the stationary measurement scatter under its assumptions; it does not establish the optimum or consistency of the complete filter, nor the best controller. Lower estimator RMSE, smoother force and better tracking are separate outcomes. The correlated-noise boundary can worsen tracking after applying the white stationary covariance estimate; covariance magnitude cannot remove temporal correlation. In the delay boundary, R calibration does not implement delayed-input prediction or change actuator timing.

These eight cases are a declared educational/development set, not a rare-failure certification or an indefinitely untouched final test. Once used for future design, subsequent measurements are regressions on known cases. Full-length aggregate comparisons use only cases completed by all arms and report excluded cases; pairwise common-prefix comparisons remain available when a future run ends early. No improvement threshold forces a green scientific conclusion. NIS and innovation lag-one are diagnostics under the nonlinear/assumed-noise model, not formal chi-square consistency admission.

### Visible integration and verification

The shared `index.html` contains the calibration experiment, not a disconnected second dynamics demo. Its live execution uses the same physics and core classes. While it runs, the original live experiment is paused without altering its controller, observer, scenario, state or default settings. The separate result area is explicitly **record replay**, with a sample scrubber and first-saturation action. Quantities have labelled m/deg/N axes; all arms use the same plot range for a given case. The histogram is centered on the sample mean, not labelled true sensor error. Invalid cases or failed computation are surfaced; no arm is silently applied to the original controller.

Commands in the existing verification workflow:

    node tests/test_calibration_lab.mjs
    npm run validate:calibration
    python tests/test_calibration_reference.py
    python tests/test_calibration_browser.py

The data-only and contract tests cover offset invariance, unavailable samples, nonwhite/correlated-input rejection, fixed Q_e/gain, clipped-force identity, row timing, sample separation and same-data replay. The reference checks all 14,400 dynamic frames, sample covariance, NIS and aggregate arithmetic. The real HTTP browser test prohibits the legacy JS physics and injected-variance getter, compares computed values against Node results, preserves the original main state/defaults and inspects a real saturation frame.

Receipts: `evidence/calibration_lesson.json`, `calibration_reference.json`, `calibration_browser.json`. Full transient traces remain in ignored `test-results/calibration_lesson_full.json`; the committed receipt keeps summaries and the small stationary dataset (dynamic frames are regenerated locally). The browser calculates a selected case live, not by playing a fabricated animation.

### Remaining educational work

This closes the measured-R causal comparison, not the whole commissioning curriculum. Process covariance/initial-state calibration, bias and timing identification, controller/observer bandwidth co-design, and an interactive stability/region-of-attraction experiment remain separate tasks. Existing identification/tuning code and native solver work should be reused when those questions are tackled, not recreated. Actual legged contact, hardware control or all-N completion are not required for this slice's success.

References: NIST repeated measurement/repeatability example https://www.itl.nist.gov/div898/handbook/mpc/section6/mpc6221.htm ; FilterPy's Q/R/K/S and predict/update definitions https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html . The external methods justify the measurement/filter definitions, not the performance numbers of this local simulation.
