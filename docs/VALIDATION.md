# Validation

## Reproduce

    npm ci
    npm run check

The check runs:
- JavaScript syntax validation,
- standalone index rebuild parity,
- deterministic engine probes.

## Current fixed probes

The checked-in test verifies:
- PID, LQR, Linear MPC, Centroidal-style MPC, Full NMPC, and PPO survive the nominal truth-state probe,
- Linear MPC exposes N=30,
- Centroidal-style MPC exposes N=32 and a downstream reference,
- Full NMPC exposes N=30 and a bounded iterative solve,
- Full NMPC stays under the 20 ms educational control slot in the fixed Node probe,
- raw finite-difference estimation fails in most seeds under the fixed strong sensor-noise probe.

More exhaustive generated measurements are stored in:

    evidence/control_observer_v2_metrics.json

Those numbers are paired CartPole evidence only.

## Demo recording

The README animation is generated from a real browser run:

    python3 scripts/record_demo.py

The recorded configuration uses:
- full nonlinear NMPC,
- Adaptive-R observer bridge,
- sensor-noise scenario,
- external pushes and goal changes.

The recording is an interface demonstration, not a benchmark.
