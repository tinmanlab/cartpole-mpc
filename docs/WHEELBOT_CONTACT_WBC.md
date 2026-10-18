# Same-contact inverse diagnostic

The original rigid-contact `build/solve/advance` functions remain unchanged as
hold/direct-pivot comparisons. They can satisfy their own rigid equilibrium while
requesting accelerations and contact forces that MuJoCo cannot realize.

The additive `ContactInverseController` optimizes only six accelerations using
SciPy SLSQP. MuJoCo 3.7.0 `mj_inverse` supplies the required generalized force and
contact law; the first three required forces must be zero. Only the final three
motor forces become controls. There is no independent contact-force decision
variable, outside force injection, TSID binary, new physics, or numerical flag
change. `mjd_inverseFD(eps=1e-6, flg_actuation=0)` supplies the transposed DfDa
Jacobian; the objective gradient is analytic. Motor, next-step joint, and 50 ms
braking bounds accompany the root equalities. Successful solver status is
mandatory, including when an iteration-limit candidate happens to be feasible.

Each candidate is independently checked with inverse and forward dynamics at
identical q/v, zero external forces, and cold warmstart before acceptance and
again before replay. Componentwise continuous qacc errors must be <=1e-6 in
m/s² for slides and rad/s² for hinges, root force <=1e-7, torque violation
<=1e-8 Nm, torque reconstruction <=1e-8 Nm, and aggregate world contact force
error <=1e-6 N, with matching ordered contacts and nonnegative normal forces.
Five 2 ms steps then use the existing 10 ms q/v state map. Within those five
steps, warmstart carries normally. The trace distinguishes preintegration force
and acceleration from postintegration state; it never substitutes finite-step
acceleration for continuous acceleration.

The frozen protocol is `tests/fixtures/wheelbot_contact_inverse.json`: 4 s gravity
settle, 0.4 s cubic reference to 0.08 rad **away from upright**, then 0.1 s hold.
Pitch feedback uses kp=25 s^-2 and kd=10 s^-1. The root-height target and its
velocity/acceleration follow the fixed head-edge pivot geometry. The hip material
edge is local (sign(pitch)*0.0625, 0, 0), and its head counterpart has local
z=0.25. Root rise, material edge rise, and absolute clearance are separate.
Acceptance requires >=5 mm edge rise and >=0.05 rad away rotation at the same
instant, with all 50 commands completed, joint excursion <=0.05 rad and
penetration <=0.02 m throughout. These are two known development poses.

Run with the existing native Python:

```
/home/tinman/cartpole-mpc-audit-20261004/.venv-native/bin/python tests/test_wheelbot_contact_inverse.py
/home/tinman/cartpole-mpc-audit-20261004/.venv-native/bin/python tests/test_wheelbot_contact_wbc.py
/home/tinman/cartpole-mpc-audit-20261004/.venv-native/bin/python scripts/validate_wheelbot_contact_inverse.py
```

The checked evidence records source/protocol/XML/profile hashes, every trial and
rejection, force prediction errors, and median/p95/max nanosecond timing with
10 ms overruns. The ignored raw trace records inverse/forward contact pairs,
q/v, controls, actual motor force, and all substep warmstarts and states. Timing
excludes settling and final evidence serialization; fullstep includes command
construction, validation, five physics steps, metrics, and trace serialization.

The executed inverse trials meet the declared displacement and physical gates,
but substantially overshoot the requested small rotation. This is a remaining
trajectory-control failure, not successful tracking or a stable hold. Evidence
separates the threshold outcome, solver acceptance, and physics agreement.
No full get-up, observer, RL, WASM parity, hardware, or hard-real-time claim is
made. No gains or thresholds were retuned after the first frozen comparison.


## Sampled command duration and bounded braking

Inverse/forward agreement holds at the state where the command is computed. It does not imply that holding that torque through several contact-changing integration steps preserves the requested acceleration. The previous0.5s minimum-lift test crossed its threshold but overshot the0.08rad reference; it was not a successful settling controller.

The explicit rate comparison in `tests/fixtures/wheelbot_contact_braking.json` keeps the physical timestep2ms, model, motor limits, reference trajectory and gains unchanged. One branch retains the original10mszero-order torque hold; a separate candidate recomputes feedback every2ms. The new candidate also explicitly permits200optimizer iterations, versus the old100. This is not a silent promotion of the10ms controller or a hardware-capability change.

Acceptance includes the final0.1s window: angle error<=0.02rad, pitch speed<=0.1rad/s, overshoot<=0.02rad, at least5mmhip-edge rise, all-substep joint excursion<=0.05rad and penetration<=0.02m. Solver iteration-limit results are rejected. Both known mirrored2mstrials passed all250commands and settled near0.080261rad with approximately20.04mmhip-edge rise. Both10mshold trials retained their earlier overshoot and failed the stronger tracking criterion. This is one small support-transfer primitive, not upright recovery or general stability.

Repeated official inverse-dynamics scratch allocation is now optional. The reused buffer is reset to preserve fresh-state semantics and is confined to a controller, not shared with the plant. Direct tests compare its forces, Jacobians and resulting command with the original allocation path. This does not change the optimizer, dynamics or default legacy controller behavior. The iteration budget is a separately validated argument, not a relaxed acceptance criterion.

The new2msfeedback branch does not yet meet a2mswall-clock deadline. Its complete observed control-cycle median was approximately9.4–10.0ms, with every250/250cycle exceeding2ms per trial. A0.5ssimulation took approximately2.53–2.62swalltime, excluding initial settling. Physics integration alone was about0.03msmedian; optimization, forward checking and diagnostic work dominate. No measured-state observer, real-time deployment or WASM online-controller claim follows from this native exact-state experiment.

Reproduce: `python scripts/validate_wheelbot_contact_braking.py`, then `python tests/test_wheelbot_contact_braking_reference.py`. Existinginverse/WBCunit tests and `tests/test_wheelbot_contact_workspace.py` cover unchanged defaults, cache equivalence and rejected-input behavior. The independent reference reconstructs600commands/1000physics steps, force output, physical limits and final-window metrics. A future faster implementation or sampled-data plan must be revalidated rather than assuming this high-rate diagnostic is already usable on hardware.

Offline contact trajectory tracking (2026-10-05): `scripts/design_wheelbot_contact_tracking.py` verifies and extracts the admitted 2 ms braking traces, whose contact inverse establishes the nonlinear motion **offline**; it central-differences the unchanged native 2 ms cold-start internal-data map and uses the finite-horizon Riccati pattern from `design_wheelbot_jump.py`, with unchanged contact-profile Q/R and terminal P = Q. `scripts/wheelbot_contact_tracking.py` applies reference lookup plus time-varying trajectory-error feedback **online**, validates state/schedule/source identities and clips torque to 16/16/1.7 Nm; it calls no inverse, derivative, optimizer or plant. The physical state remains continuous after the explicit initial condition, and scratch MjData resets preserve the source map's zero-warmstart semantics. This is not the old standing LQR: each gain belongs to its nonlinear reference sample. The locked assessment compares feedback and torque replay on both signs, each nominal and with hip offsets ±0.005 rad or body-rate offsets ±0.01 rad/s; these physical contact perturbations are deterministic tests, not statistical robustness or parameter/model transfer. Runtime state is exact simulator state; measured-state KF, hardware operation, global getup and guaranteed safety are not implemented. The original two-epsilon derivative-validation gate fails and remains reported as a native prototype limitation, despite successful bounded trial metrics; a single smaller-epsilon diagnostic is recorded without changing gains. The profile, raw derivative/replay/trial records and separate command/physics/diagnostic/loop timing are bound in `evidence/wheelbot_contact_tracking.json`; offline build time and unverified hard-real-time status are explicit. The prior 9–10 ms inverse cycles used different logging, so that comparison is not like-for-like.

Selected-derivative admission correction: `branchDiagnostics.selected_derivatives_validated` now separately requires every selected 1e-7 column to remain on the nominal contact and constraint branch, and either a same-branch coarse comparison below 0.1 or an exact `(sign, step, kind, column)` smaller-step record with both branches nominal and error below the same 0.1 threshold; missing, duplicate, unmatched, nonfinite or poorly conditioned evidence is rejected. The runtime requires an explicit true admission flag at construction (this is not cryptographic verification). The original coarse gate remains false, with crossed-branch/excessive-step differences excluded as valid derivative evidence; all numerical arrays and locked assessment values remain unchanged. This establishes finite local consistency only, not stability through contact changes: the 2.237 rad/s perturbed wheel-velocity prediction discrepancy remains. The historical 200-iteration budget applies solely to offline inverse reference generation; the current exact-state online tracker has no solver, observer, hardware or terminal-standup claim. Prior artifacts are preserved under ignored `test-results/contact-tracking-pre-admission-*`; the coordinator reference check remains incomplete and is separate from the passing native/replay tests.

Shared WASM integration (2026-10-05): the independent reference checker is now
closed: its pure-command import whitelist admits standard-library `hashlib`,
while rejecting MuJoCo/SciPy imports. All 20 native replays (5,000 steps), gain
reconstruction and unchanged physical/task criteria pass. The shared physical
editor offers explicit **Reset & lift/hold · left/right**, only for the exact
hashed default full-contact XML and admitted profile. This is the first ~2 cm,
4.6 degree hip-edge support transfer, not standup. It uses exact simulator state,
not an observer, and an offline plan plus scheduled feedback, not online MPC.

`src/wheelbot_contact_tracking.mjs` validates the profile/source identities once,
then performs reference lookup, 3×11 feedback and logged torque clipping. Each
command advances one actual official MuJoCo WASM 2 ms step. The contact wrapper
has two MjData objects: fast trials step only its formerly diagnostic data; the
historical core supplies read-only geometry and continues to integrate ordinary
10 ms modes. Internal data is cold-reset each command while physical qpos/qvel
remain continuous; the only initial-state reset is the explicitly labelled setup.
The original `src/wheelbot_backend.mjs` remains byte-identical. Rendering occurs
outside feedback, with a 40 ms catch-up budget; no browser deadline is claimed.

Run `node tests/test_wheelbot_contact_tracking_wasm.mjs` for all 20 frozen-native
comparisons, source/expiry rejection and editor admission tests. Results are in
`evidence/wheelbot_contact_tracking_wasm.json` and the independent native result
in `evidence/wheelbot_contact_tracking_reference.json`. Feedback passes 10/10;
torque replay passes 2/10. `tests/test_wheelbot_contact_tracking_browser.py`
covers the shared buttons, manual stepping, restoration, mismatch and screenshot;
it must run in normal CI and was not executed in the socket-restricted workspace.
