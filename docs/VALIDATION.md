# Validation

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
