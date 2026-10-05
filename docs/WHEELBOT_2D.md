# Planar wheel-leg transfer experiment

This is a second physical asset for the CartPole commissioning lesson, running the same pinned MuJoCo 3.7.0 WASM in the browser. It is a simplified single-side sagittal derivative of Upkie and the user sketch, not a full Upkie reproduction, hardware digital twin, or demonstration of one-wheel 3D stability. Out-of-plane motion is suppressed by the model's joint coordinates.

The sketch supplies the topology and colours: a white rectangular torso, two blue links, two red actuated hip/knee pivots and one green motorized wheel. Exactly three joints are actuated. The drawing supplies no dimensions, masses, torque limits or sensors.

## Lineage and assumptions

`assets/wheelbot/source/upkie.urdf` and `LICENSE` pin official `upkie/upkie_description` commit `94735fbe6137276a41de0ff4cc04d2e533fa9e33`. The original URDF SHA256 is `d15965215067276203599a850e5c7ebb319ed6815506f0a2721dae78abac2483`. [Original source](https://github.com/upkie/upkie_description/blob/94735fbe6137276a41de0ff4cc04d2e533fa9e33/urdf/upkie.urdf). Source-derived lengths are 0.25 m per link, tire radius 0.05 m, and motor limits 16/16/1.7 Nm. The generator and generated provenance identify the extracted masses and dimensions and every simplified primitive assumption. No STL/tire mesh or restrictive third-party biped source is used. The copied URDF refers to upstream meshes for provenance only; the standalone MJCF loads none of them.

Primitive mass distributions use analytic inertias, replacing upstream inertia tensors rather than claiming their validation. Contact parameters, numerical timestep, actuator armature/damping, sensor noise, tuning costs, trim pose and test envelopes are declared teaching assumptions. This small contact experiment is local: torso/link impacts and arbitrary recovery poses are outside its admitted operating envelope.

## Physical and measurement contract

The full state is `[x,z,pitch,hip,knee,wheel,vx,vz,pitch_rate,hip_rate,knee_rate,wheel_rate]`. x/z are torso-root world translation (m), angles are relative hinges (rad), and rates are m/s or rad/s. Inputs `[hip_motor,knee_motor,wheel_motor]` are joint torques (Nm), with unit transmission gear. The unactuated root has x/z slides and a pitch hinge; all hinge axes are world sagittal y. Floor friction contact supplies traction. There is no equality connecting wheel angle to x, no welded support, and no horizontal actuation substituted for wheel torque. The explicit `externalX` argument is a separately reported disturbance force in N.

Each control sample advances five 2 ms Euler steps (10 ms). Geometry comes from compiled MuJoCo site/geom transforms. Contact loss is counted; signed slip is wheel-center horizontal speed minus radius times total absolute wheel angular speed, in m/s. It is a kinematic contact-point slip diagnostic, including while airborne, not a no-slip constraint.

The controlled state removes only wheel absolute phase: full-state indices `[0,1,2,3,4,6,7,8,9,10,11]`. World position and wheel speed remain, preserving actual slip dynamics. Wheel-phase invariance is checked locally; a local linearization/rank result is no global stability guarantee.

Offline Python uses native MuJoCo finite differences, SciPy least-squares trim and discrete Riccati solutions. The profile binds gains, matrices, qref and uref to the XML SHA256. The browser validates model mappings, profile dimensions/hash and design acceptance before enabling simulation. It runs WASM, not Python. The initial state uses genuine trim qref; the initial filter receives noisy position measurements and zero velocity prior.

The browser feedback modes are local saturated LQR or three-input torque-constrained linear MPC, each using the same stationary linear KF. The filter predicts deviations with A/B and applied torque, then corrects using noisy x/z localization, torso orientation, hip and knee measurements. True velocities are never sensor inputs. External x/z localization is an explicit teaching channel: this is not an IMU-and-encoders-only feasibility claim. A wheel encoder cannot establish absolute x under slip. Passive zero-torque dynamics are available for comparison. The displayed reference is local and the controller is not a swing-up or global recovery policy.

## Explicit algorithm boundary

Browser MIMO MPC is implemented by the separate `src/wheelbot_mpc.mjs` adapter. `src/engine.js` / `src/qp.js` and existing CartPole algorithms remain unchanged: their historical first-B-column behavior is still tested and is never used for wheelbot. Unknown `nmpc` mode is rejected. Independent sparse OSQP arithmetic checks cover the new three-input adapter. No EKF is implemented or advertised.

## Reproduction and evidence

Use the existing native Python environment (MuJoCo 3.7.0, SciPy 1.15.3). For the original model run the generator and offline design script, then `npm run test:wheelbot` before `python tests/test_wheelbot_reference.py` (the native test consumes the exported WASM replay). Actual HTTP behavior is checked by `python tests/test_wheelbot_browser.py` in the existing permitted CI environment. Serve the repository over HTTP and open `wheelbot.html`; it remains paused after validation. `npm run check` still verifies the original lab.

The fixed case manifest is `tests/fixtures/wheelbot_cases.json`: local balance, position regulation, small push and large boundary push, each with three seeds. Completion and task success are distinct. Final 50 samples must meet 15 mm position and 0.025 rad pitch thresholds. Failures are recorded, not retuned away. Read `evidence/wheelbot_validation.json` and native/browser receipts for measured outcomes. These finite cases do not certify robust stability or hardware transfer.

The declared `home` MJCF keyframe shows hip +0.15 rad and knee −0.30 rad with the wheel at the floor for generic viewers. It is a display pose, not a balanced dynamic reference. The profile uses native least-squares to solve height, pitch and three feedforward torques at those fixed joint angles; full precision is retained. Root damping/armature are zero. The total selected primitive mass is 1.19915 kg; omitted motor hardware makes this substantially lighter than a complete robot.

Current verification: the fixed 12 WASM trials completed 9 and passed the task thresholds in 3. This is a measured bounded result, not universal local transfer. Independent native replay covers 160 control samples with contact and airborne segments. A separate +20% torso-mass, feedforward-only probe measures trim sensitivity; it is not a closed-loop mismatch guarantee. Browser HTTP testing is BLOCKED in this workspace: Python socket creation returned `PermissionError: [Errno 1] Operation not permitted` (exit 1). No browser screenshot or browser pass is claimed; the Playwright test remains available for normal CI.

Reproduce numerical evidence in order:

```
python3 scripts/build_wheelbot_model.py
python scripts/design_wheelbot_profile.py
npm run test:wheelbot
python tests/test_wheelbot_reference.py
```

Raw Node states, inputs, estimates, measurements and contact diagnostics are exported to `test-results/wheelbot_wasm_reference.json`; compact receipts are in `evidence/wheelbot_validation.json` and `evidence/wheelbot_native_reference.json`. The nonlinear finite-difference derivative check is independent of `mjd_transitionFD` composition; NumPy verifies the JS controller and KF mean equations from the same supplied inputs. The fixed test cases and gains were not searched or retuned to improve the score.

Measured fixed-case outcomes (three seeds each): local balance completed/passed 3/3; position completed 3/3 and passed 0/3; small push completed 3/3 and passed 0/3; boundary push completed/passed 0/3, with failure at step 105 after four contact-loss samples in each seed. Position convergence is too slow for the fixed 3-second threshold; no tuning search was performed. See `evidence/wheelbot_commands.json` for exact command exits and explicit not-run checks. These are historical baseline outcomes. The new MPC and native sparse-QP checks are described below. EKF, closed-loop mass-mismatch campaigns and hardware validation remain unsupported or not run.

Review correction: errors are reported per state/measurement channel with physical units, not as a single mixed-unit score. The native reference reconstructs every task/step/contact/saturation count from raw traces. The 2D camera follows the torso, with world-position ticks; wheel slip uses m/s and all 11 estimated states are displayed. Browser assertions require 20 actual valid physics steps, not merely any partial number below 20.

## Three-input local linear MPC

`src/wheelbot_mpc.mjs` transcribes the standard finite-horizon state/input
problem to pinned MIT quadprog 1.6.1. It uses every B column and full Qc (the
profile's `Q`), Rc (`R`), and Riccati terminal P. N=20 is 0.2 seconds, with
60 decision entries. The original SISO adapter and plant are unchanged.
The exact coordinate change du=-K e+v retains all finite-precision cost terms;
fixed matrices are cached only in each controller instance. The conditioning
receipt compares this transcription with direct condensing.

Bounds apply to actual hip/knee/wheel torque: -limit-uref <= du <= limit-uref.
World-x reference translation is checked both through A[:,0]=unit_x and a
translated nonlinear plant step. Wheel phase remains omitted. There are no
state, contact, or friction constraints: this is neither contact NMPC nor
robust MPC. Riccati terminal P makes inactive-constraint MPC agree with the
same LQR design; changing controller labels does not improve that design.
Qc/Rc are control preferences; existing Qe/Re and noisy localization/KF inputs
remain noise assumptions, with no auto-calibration claim.

The adapter rejects bad dimensions, nonfinite data, model/limit mismatch,
solver failures, KKT/complementarity over 1e-7, primal or original physical
bound/dynamics defects over 1e-8, and original-unit stationarity over 1e-7.
Rejected solutions never advance the trial; no clipped-solution fallback is
used. Very large offline constructed states can fail these strict numerical
gates. Constructed active-bound tests are algebra checks, not achievable plant
initial conditions.

Independent validation assembles the full sparse state/input QP in SciPy and
OSQP, without importing the JS condensed Hessian. It checks N=1 and N=20,
inactive and all-three-input active bounds, full off-diagonal Rc, trajectories,
objectives and actual first commands. See `tests/fixtures/wheelbot_mpc_validation.json`.
The formulation follows the [official OSQP MPC example](https://osqp.org/docs/examples/mpc.html)
and the [quadprog convention](https://github.com/albertosantini/quadprog)
(.5 v'Hv-d'v, inequality columns C'v>=b; pinned runtime is one-indexed).

Original 4-case × 3-seed regression: both baseline LQR/KF and MPC/KF complete
9/12 and meet 3/12 tasks, with zero QP rejections. The three physical envelope
failures remain in evidence. Existing baseline final states are checked for
exact numeric equality. New seeds 101,211,307 use the same four physical cases;
they are independent noise seeds, not new physical OOD. Receipts separate
completion, task success, QP rejection, physical envelope failure, contact loss,
saturation, physical-unit estimation RMSE, tail errors, per-motor torque/slew,
and host timing. Host timing is not a real-time promise.

Browser tests exercise actual LQR and MPC numerical loops, shared estimator
initialization, reset and rejection paths, and preserve the screenshot step.
Local browser execution was prohibited by the sandbox task; GitHub CI must
supply that verification. No browser performance claim is made.

## Model-response design correction

The earlier global-monotonicity rejection was a conservative policy, not
mathematical impossibility. Brent's method needs continuity and a sign bracket,
not global monotonicity. The specified maximum absolute error over samples
250..300 is bracketed on log10(Qx multiplier) in [0,4]. Model-only SciPy DARE
and Brent evaluation returned multiplier 16.0513257776, with maximum error
0.01000000000003 m for the 0.03 m full-state step. The residual is within the
recorded 1e-10 m numerical tolerance. Thirteen actual model evaluations include
ten Brent function calls. Exact source, script and response-profile hashes,
endpoint residuals, root diagnostics and tail prediction are in
`evidence/wheelbot_response_design.json`.

`assets/wheelbot/response_profile.json` changes only Qc[0,0], P, K and associated
control-design diagnostics/metadata. A/B, Rc, trim, KF gain, Qe/Re, sensor noise,
physical model, torque limits, task thresholds and episode lengths are unchanged.
This is model-based one-parameter design, not nonlinear/controller optimality.
The explicit UI selector starts at baseline; neither MPC nor the response
profile is promoted by default.

The response choice was locked before its nonlinear evaluations. Seeds
101,211,307 had previously been run for baseline-only evidence; they were not
read by the model-design script or used for weight adjustment. The same physical
cases are reused, so this is independent noise, not physical OOD. The original
24-run regression receipt is retained unchanged. The new 48-run comparison is
in `evidence/wheelbot_mpc_response_validation.json`; raw fixed traces are retained
in ignored `test-results/wheelbot_mpc_response_traces.json` and independently
recomputed with NumPy/SciPy.

Each baseline controller completes 9/12 and meets 3/12 tasks; each response
controller completes 9/12 and meets 6/12. All three position cases now meet the
unchanged task tolerance; small push and boundary push still fail the task.
Each group retains three physical-envelope failures and 12 contact-loss samples.
The initial comparison had zero QP rejections or applied-torque saturation
samples. Horizon-wide bound activity and same-estimate action differences are
now measured separately; see the current machine receipt rather than inferring
constraint inactivity from applied commands alone. Separate offline
constructed probes exercise all three bounds and compare constrained actions;
they are not evidence of physically reachable initial conditions. The larger
constructed state remains a recorded numerical rejection at unchanged gates.

One recorded development pass measured 0.30–0.34 ms per LQR step and 4.33 ms per MPC step, with a maximum MPC solve of 35.05 ms. Regeneration changes host timings. The 10 ms control interval is simulated time, not a measured worst-case execution guarantee; current timings are in the machine receipt.

### Final review boundaries

The nonlinear assessment seeds 101/211/307 were not used by the model-only response-design function. Their baseline runs were nevertheless observed during adapter verification before the corrected response root was finalized; they are development comparison data, not a strictly unopened final holdout. All reused task cases are regression cases. A fresh uncertainty/hardware claim requires separately reserved assessment.

Reporting distinguishes the applied first torque reaching a bound from any torque bound active anywhere in the MPC horizon. Absence of a saturated applied input alone does not establish absence of active predicted constraints. Same-estimate LQR/MPC differences cover every executed sample, not only saturation events. Zero executed samples have unavailable/null error/timing extrema; unexpected implementation exceptions abort the experiment instead of being mislabeled a QP rejection. An unsupported UI mode request pauses and clears the active trial.

Reproduce the added comparison after the baseline model/profile are available:

```
python scripts/design_wheelbot_response.py
npm run test:wheelbot:mpc
python tests/test_wheelbot_mpc_reference.py
node scripts/validate_wheelbot_mpc.mjs --new-seeds --response
python tests/test_wheelbot_response_reference.py
```

The response model is optional and selected explicitly; it is not written over the baseline profile. The generated response function reproduces only a nominal model target. Nonlinear sensor-in-the-loop assessment is the separate comparison above, with its observed seed-reuse limitations.

Optional response profiles are checked against the exact raw-byte SHA-256 of the baseline fetched by the page. The loader independently checks the declared one-variable Qx change and unchanged model, observer, noise, trim and limits. A stale source hash or changed noise/observer field disables only the response option; baseline operation remains available. This is accidental stale-data validation, not a security signature or independent stability certificate.

## Local disturbance recovery and runtime reuse

The optional `recovery_profile.json` is a second model-designed Qx-only profile, not a replacement for either historical profile. `scripts/design_wheelbot_recovery.py` derives the horizontal-disturbance column from central differences of the same five native MuJoCo substeps. The offline design simulates the existing stationary KF mean together with the nominal linear plant. The actual disturbance is never supplied to the online controller or filter.

The frozen target includes both the 3 cm reference step and the 3 N, 0.1 s pulse: maximum position error 10 mm and pitch error 0.020 rad over samples 250–300. Brent root finding in the fixed log10 factor bracket [0,4] gives Qx multiplier about 1375.03647 relative to the original Qx=2. This is one declared nominal design requirement, not a global optimal parameter, an estimator calibration or a hard state constraint. A/B, Rc, L, Qe/Re, initial state, model, limits, sensor noise and the original test thresholds are unchanged.

The profile and `tests/fixtures/wheelbot_recovery_validation.json` were fixed before opening noise seeds 401/409/419. These are new noise realizations on known physical cases, not new physical OOD. The 72-run assessment compares baseline/response/recovery with LQR/KF and MPC/KF. Each recovery controller completes 9/12 and meets 9/12 tasks; each old response controller meets 6/12 and each baseline controller 3/12. The small-push task now passes 3/3 for each recovery controller without losing its balance or position-task passes. No force/noise/time/task-limit relaxation is used. All three 40 N boundary failures per group remain failures.

The native reference independently reconstructs the joint 22-dimensional plant/filter mean model, Riccati gains, seeded filter initialization, every recorded KF mean, original task metrics and native one-step dynamics for all 18,090 executed frames. `evidence/wheelbot_recovery_reference.json` records those results. Native contact-force diagnostics from the actually applied commands are in `evidence/wheelbot_contact_diagnosis.json`; they are not online sensor inputs. In the recorded 40 N onset, support force vanishes at step 102 and the pitch envelope fails at 105. The filter substantially underestimates the rapid pitch rate. This identifies a model/estimation regime change, not a proof that every controller must fail. Static mu*m*g is only a scale comparison and is not used as a dynamic infeasibility certificate. Hybrid contact-aware estimation/control and large-pulse recovery remain unimplemented.

MPC runtime changes cache fixed transposes, one-indexed representations and the linear map from initial error to the complete finite-horizon objective. The upstream quadprog call still runs for every solve; its mutable inputs are copied, not reused after mutation. Full Rc/P, every B column, tiny nonzero cost terms, original-unit KKT/dynamics/torque gates and rejected-command no-advance behavior remain intact. The paired timing receipt reports distributions and outliers, not a hard real-time guarantee. One development comparison lowered baseline N20 p50 from 2.188 to 0.454 ms and response p50 from 1.060 to 0.413 ms; response p99/max did not improve in that sample. Current measurements are in the receipt.

The browser redraws once per animation frame instead of once per control tick. Its wall-clock catch-up budget is bounded to four unchanged 10 ms control samples; discarded wall catch-up is displayed. This prevents an unbounded display backlog, but does not skip physical integration steps or establish hardware deadline compliance. Pauses and slow browser frames do not change the physics timestep.

Reproduce the added assessment (use the existing native Python environment):

```
python scripts/design_wheelbot_recovery.py
node tests/test_wheelbot_runtime.mjs
node scripts/validate_wheelbot_mpc.mjs --recovery
python tests/test_wheelbot_recovery_reference.py
```

The optional paired performance test accepts an exact saved pre-change `src/wheelbot_mpc.mjs` as its first argument. Timing includes host/VM scheduling and garbage-collection effects. The recovery selector remains opt-in and explicitly does not claim contact-loss recovery.
