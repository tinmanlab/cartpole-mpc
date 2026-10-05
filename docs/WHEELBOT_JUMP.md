# Motor-driven wheelbot jump

This experiment uses the canonical nonlinear MuJoCo wheelbot: six coordinates,
three motor torques (hip, knee, wheel), 2 ms physics and 10 ms control. It extends
the SISO CartPole teaching context to coupled MIMO actuation; CartPole success
is not evidence for this plant. The 1.19915 kg primitive model excludes motor assemblies and electronics; ideal torque actuation and source-derived limits are engineering assumptions. There is no hardware validation. The learning scope is static balance, 3 cm position regulation, a model-specific ±0.8 N × 0.2 s hip-point push, and this verified planned jump.

The offline generator constructs a nonlinear motor trajectory using true state.
Runtime tracking is finite-horizon time-varying LQR with a scheduled linear
Kalman filter, **not online NMPC**. Its five noisy measurements are x, z, pitch,
hip and knee; all six estimated rates start at zero. Truth is used for plant
simulation and evaluation, not feedback. External force is zero throughout the
jump; external-force calibration belongs to the separate recovery experiment.

The original standing Q diagonal is `[2,30,80,15,15,.6,8,4,1.5,1.5,.25]` and R
is `[.08,.08,.35]`. Jump tracking uses the existing recovery Q/R and terminal P
(Qx = 2750.072941591512), without new tuning. Sensor standard deviations remain
`[.001,.001,.002,.003,.003]`; Qe/Re come from the baseline. Torque limits remain
`[16,16,1.7]` Nm. Offline posture parameters remain 0.15 s standing, 0.8 rad
crouch over 0.65 s, 0.22 s extension, Kp=120, Kd=22, position-to-pitch=-0.5,
and velocity weight=0.5 s. Local differences use 1e-7/1e-8 with matching
plus/minus contact sequences. This is local linearization, not a guarantee
that disturbed motion follows the nominal contact branch.

From the repository root, use the checked-in vendor runtime with Node 22 and
an existing Python environment containing MuJoCo 3.7.0, NumPy and SciPy:

```sh
mkdir -p test-results
node tests/test_wheelbot_jump_contract.mjs
node tests/test_wheelbot_jump.mjs assets/wheelbot/jump_profile.json test-results/jump_final_wasm
/home/tinman/cartpole-mpc-audit-20261004/.venv-native/bin/python tests/test_wheelbot_jump_reference.py
```

Elsewhere, replace the Python executable with an equivalently provisioned one.
The reference check consumes the preceding WASM trace and writes
`evidence/wheelbot_jump_finish_reference.json`, preserving prior evidence.
Do not regenerate the profile merely to run verification. The generator is
`scripts/design_wheelbot_jump.py`; regeneration changes provenance hashes.
Normal CI with Playwright can run `python3 tests/test_wheelbot_jump_browser.py`.
It starts HTTP itself, checks actual integration counts, saves airborne/landed
screenshots, checks completion/reset, and rejects a mismatched optional profile.
Browser execution was not permitted in this sandbox; screenshots are not fresh
verification here. That test still compares against the prior
`evidence/wheelbot_jump_reference.json`, checking its profile hash.

Fresh WASM and independent native replay on 2026-10-05 produced:

| Noise seed | Control steps | COM rise (mm) | Flight (ms) | Tail x error (mm) |
| --- | ---: | ---: | ---: | ---: |
| 809 | 400 | 66.821 | 250 | 3.913 |
| 821 | 400 | 65.989 | 240 | 3.548 |
| 823 | 400 | 66.140 | 250 | 3.889 |

All three meet the unchanged protocol: at least 10 mm COM rise, 5 mm wheel
clearance for at least 30 ms, then 0.5 s grounded settling below 15 mm x,
0.025 rad pitch and 0.03 rad hip/knee errors. Nominal torque replay fails at
step 278; its flight does not establish stable landing. Across all four traces,
native replay checked 1,478 control / 7,390 physics steps; maximum one-step state
component discrepancy was 2.94e-12 and estimated-state discrepancy 1.31e-13.

Euler COM momentum has an observed maximum local residual of 0.0132312 Ns.
At the worst sample, halving dt from 2 to 1, 0.5 and 0.25 ms reduces the residual
to 0.00309822, 0.000745588 and 0.000182587 Ns. This is local refinement evidence,
not exact momentum conservation or a global convergence proof.

Evidence binds profile SHA-256
`e7a15161d481b7d23158bf0dd70885742a4fc8665523e1203fc75e8e5d11d0b1`
and original model SHA-256
`fe62740619c7fdc2a744a27ffcb4b4589cedee5cf3af39c404e66198faa6961a`.
The profile binds baseline, protocol and generator hashes; it does not contain
a recovery-source hash. Native verification recomputes gains from the current
recovery source, but adding an explicit source binding requires coordinated
profile regeneration. The assistant-introduced 40 N experiment is archived, not an owner requirement or release blocker; see [normal-operation scope](WHEELBOT_FORCE_ENVELOPE.md).
