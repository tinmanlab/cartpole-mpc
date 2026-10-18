# Native MJPC wheelbot diagnostic

This adapter compiles and executes the official sampling planner at
`ff572a21e7c2bf9fda62e1862a758da7e9a8719b`, read from
`/home/tinman/repos/mujoco_mpc`. It does not implement another optimizer.
Scientific get-up success is separate from successful source adoption. See the
unique `evidence/wheelbot_mjpc_*.json` receipts for actual outcomes, including failures.
No observer integration or WASM parity is claimed.

## Reproduce

From the repository root, using the already installed native environment:

```sh
python3 tests/test_wheelbot_mjpc.py
python3 scripts/mjpc_wheelbot/build.py
/home/tinman/cartpole-mpc-audit-20261004/.venv-native/bin/python scripts/mjpc_wheelbot/run.py standing .05 0
/home/tinman/cartpole-mpc-audit-20261004/.venv-native/bin/python scripts/mjpc_wheelbot/run.py right 6 0
/home/tinman/cartpole-mpc-audit-20261004/.venv-native/bin/python scripts/mjpc_wheelbot/run.py right 6 1
```

Each run has a 300-second wall cap. Build logs, exact commands, generated XML,
native executable, and CSV trajectories live under ignored `test-results/mjpc-adopt/`.
Receipts are unique; older failures are not overwritten. No downloads, servers,
GUI, gRPC, services, external forces, or physical model edits are involved.

## Source and compatibility

`build.py` rejects a wrong pin or tracked upstream modifications, hashes the ten
compiled upstream source files, and verifies MuJoCo header/runtime version
`3007000` (3.7.0). The executable repeats that assertion before loading a model.
GCC compiles the source; Clang 13 with LTO links the existing Abseil archives
because they contain LLVM bitcode. Those archives have no MuJoCo ABI. No old
MJPC or MuJoCo 3.2.3 binary is linked. Function/data section garbage collection
removes unused utilities; no whole-project build is performed.

The single `compat.h` shim supplies `nullptr` for the added `mj_ray` normal
output. The installed official 3.7 `mujoco.h` explicitly marks that output
nullable (lines 665–672). The first unshimmed compile and the GNU-linker failure
are retained in build receipts. Upstream source is unchanged.

Credit: the Task/ResidualFn structure follows DeepMind Technologies' MJPC
`mjpc/tasks/humanoid/stand/stand.{h,cc}`. Upstream is Apache-2.0; see
`/home/tinman/repos/mujoco_mpc/LICENSE`. The adapter uses the wheelbot's six
velocities directly, with none of the humanoid's floating-base indexing or foot
assumptions. The optional staged support target is a small Ascento-inspired
fold/roll/sit → balance heuristic, not a reproduction of an Ascento controller.

## Physical and control contract

`run.py` checks the existing trim profile's XML hash before use. It adds only
custom planner/trim numeric values and seven user sensors to a generated copy of
`assets/wheelbot/contact_model.xml`. It compares 414 numeric plant fields,
including all numeric arrays except sensor/custom/name/address metadata, plus
all numeric solver options. Masses, inertia, collision geometry/masks, obstacle,
actuator limits, damping, armature, and 2 ms timestep remain identical.

Residuals penalize sin/cos pitch error, root/hip height normalized by the actual
trim height, hip/knee pose, horizontal displacement from the initial position,
all six velocities, normalized controls, and physical joint-limit excess.
Optional staging uses a 1.2-second support target (height .20 m, signed pitch
.8 rad, hip 1.1 rad, opposite knee 2.3 rad), then the standing trim. This changes
only task targets, never states or forces. It is one explicit heuristic, not an
outer search. Upstream random sampling is stochastic and is not seed-controlled.

Fallen starts use q=[0,.7,±1.1,.15,-.3,0], zero velocity/control, and 2000
uninterrupted gravity steps. Both sides and initial nonpenetration are recorded.
The planner receives privileged full state. It performs one optimization per
10 ms (64 trajectories, four threads, 501 states/500 physics steps, linear
spline with 24 knots). The selected spline is evaluated at **every 2 ms substep**;
this is not the existing UI's 10 ms held-action controller.

CSV rows contain pre-step time, qpos, qvel, three controls, three actual actuator
forces, contact count, penetration, limit excursion, predicted cost and iteration.
Apply each row's controls for one 2 ms step; the last row is terminal observation.
Initial state and settling recipe are in the receipt. Fresh native execution of
the first winning spline is compared against its entire upstream rollout.
WASM replay and coordinator-owned final parity remain required before integration.

## External acceptance

The Python evaluator is independent of Task costs. It requires a complete six
seconds; throughout the final second, height >90% standing trim, pitch error
<.08 rad, hip/knee errors <.12 rad and every generalized speed <.3 (m/s for
slides, rad/s for hinges). Across the trajectory: actual torques ≤16/16/1.7 Nm,
penetration <.02 m and joint-limit excursion <.05 rad. Finite traces are required.
A low predicted cost alone never passes this gate. Failure is not proof of
physical impossibility. Browser/hardware and observer validation are absent.

## Recorded adoption result (2026-10-05)

Both six-second right-side trials completed 600 optimization iterations (38,400
candidate rollouts each) with finite bounded three-actuator controls. Neither
passed get-up. The simple objective peaked at .377783 m and ended at .246012 m;
maximum penetration was .026407 m and joint excursion .369073 rad. Staging ended
at .211998 m with low speed (.011890 units/s), but remained folded: maximum tail
joint error 2.278413 rad and maximum trial limit excursion .234875 rad. Contact
counts ranged from zero to seven. This is a local-objective failure/plateau under
the bounded sampler budget, not evidence of physical impossibility or a ready
get-up controller.

Run original-plant replay with:

```sh
/home/tinman/cartpole-mpc-audit-20261004/.venv-native/bin/python tests/test_wheelbot_mjpc_replay.py
```

All three recorded traces replayed with zero qpos/qvel error on the original XML.
Replay must reproduce the settling history, including solver warm-start state;
an initial qpos/qvel-only replay failed (maximum divergence 35.08), and is not a
valid exact parity procedure. The corrected check reconstructs the recorded
initial pose and all 2000 gravity steps. WASM replay needs the same history.
No full repository/browser check was run; coordinator-owned integration and
final WASM parity are still outstanding.

## Numerical consistency correction (2026-10-05)

New native runs default to `--numerics reproducible`: the generated task XML
sets `<option><flag warmstart="disable"/></option>`. `--numerics legacy` retains
warmstart for diagnosis. The source contact XML is byte unchanged. Every numeric
mechanical array and solver option is compared, allowing only the named
`mjDSBL_WARMSTART` disable bit plus existing task/sensor metadata. This is a
mechanically identical model with a different numerical initialization profile;
it is not a claim of identical physics bytes or cross-architecture bit identity.
Runtime XML, original XML, numerical snapshots, source and binary hashes bind
receipts. The executable rejects profile/metadata/flag disagreements.

The original first-plan/qpos-qvel check was insufficient: reused upstream rollout
data retained warmstart information not included in the planner state copy.
Every replan now replays the winning policy from a fresh same-qpos/qvel/time
state over all 500 steps (1 s), compares policy controls with recorded best
actions (1e-12 Nm), and separately bounds position/angle differences at 1e-9
m/rad and velocity differences at 1e-8 m/s or rad/s. A rejected reproducible
candidate exits nonzero before applying its torque. Each executed 2 ms step
is also checked against the winning prediction, including all five steps per
10 ms replan. CSV diagnostics forward a copy, preserving live integration
history. Counts, maxima, rejection reasons and tolerances appear in stdout JSON.
Two small sidecars capture complete `mjSTATE_INTEGRATION` initial state and the
first five predicted transitions/actions for independent native/WASM checks.

Numerical consistency and physical get-up acceptance remain separate. The
six-second height, pose, joints, speed, penetration, joint-limit and torque
gates are unchanged. An exit code of zero alone is not get-up success.
Privileged state remains in use; observer integration and WASM transfer remain
unverified. No hardware, sensor, phase-cost or trajectory search changes are
part of this correction. Historical legacy receipts retain their old meaning.

```sh
python3 scripts/mjpc_wheelbot/build.py
python3 tests/test_wheelbot_mjpc.py
/home/tinman/cartpole-mpc-audit-20261004/.venv-native/bin/python tests/test_wheelbot_mjpc_task.py
/home/tinman/cartpole-mpc-audit-20261004/.venv-native/bin/python scripts/mjpc_wheelbot/run.py perturbed .2 0 --variant safe_compact --numerics reproducible
/home/tinman/cartpole-mpc-audit-20261004/.venv-native/bin/python scripts/mjpc_wheelbot/run.py right 6 0 --variant safe_compact --numerics reproducible
/home/tinman/cartpole-mpc-audit-20261004/.venv-native/bin/python tests/test_wheelbot_mjpc_consistency.py evidence/wheelbot_mjpc_consistency.json
```

## Local admission gate (2026-10-05)

The adapter now requests the same samples through the public RankedPlanner
`OptimizePolicyCandidates`, ranks their unchanged task costs, and independently
rolls each candidate over all 500 steps. Every state must be finite, joint-range
excursion ≤.05 rad, penetration ≤.02 m, and all three actual torques finite and
within the original 16/16/1.7 Nm limits. A finite first action is insufficient.
This is finite-model horizon validation, not a robust safety certificate.

Inside a declared local standing domain, the existing profile's complete 3×11 K
and uref provide another candidate. The domain is |x,z error|≤.1 m,
|pitch error|≤.1 rad, |hip/knee error|≤.15 rad, non-wheel speeds≤1 in their
respective units, and |wheel speed|≤2 rad/s (.1 m/s rim speed). The initial
1 rad/s wheel assumption rejected a measured successful LQR transient at 10 ms;
that failed receipt remains available. These are test admission assumptions,
not a proven region of attraction. No gains or actuator limits were changed.

LQR is independently rolled using the official Trajectory and Task cost path,
with actual feedback recomputed and held for each 2 ms. It must pass the same
full-horizon physical validation before selection. The lower-cost admissible
candidate is selected. CSV `selectedpolicy` and receipt `sample_selected` /
`backup_selected` expose the source; LQR-only success is not sampling MPC success.
`cost` and `lqr_cost` use the same upstream residual/norm/return calculation.
CSV also records rejected/feasible sampled counts, maximum normalized physical
constraint value, terminal position/angle error (wheel phase excluded), and
selected-plan q/v replay error. `lqr_cost=-1` means no admissible backup.
Outside the domain, if every sampled candidate fails, `NO_ADMISSIBLE_PLAN`
returns nonzero before a new plant input is applied. Earlier failed traces stay
intact. All experiments use exact state; no observer or get-up claim follows.

Raw runs now live in `test-results/mjpc-admission/`; each native child has a
300-second timeout. The acceptance test independently rolls both ±.02 rad
baselines for six seconds and exercises the native late-horizon rejection
predicate. The consistency checker rejects false/zero/inconsistent counts,
nonfinite/excess residuals, altered tolerances, initial-state disagreements,
and malformed recorded time steps. Historical sources are checked against
hash-bound build snapshots when the current adapter has changed.

## Candidate admission and an explicitly identified local backup

The adapter now obtains ranked candidates through the unchanged upstream `OptimizePolicyCandidates` interface. Before a candidate can be selected, its full nonlinear horizon is checked against the existing torque, joint-excursion and contact-penetration limits. A low cost or a valid first torque does not excuse a later violation. In the declared standing neighborhood, a candidate also has to remain in that neighborhood; this finite model check is not a certified region of attraction.

The already-designed full-contact LQR is separately rolled out through the same upstream trajectory/cost implementation. It is eligible only in the declared local domain and after its complete nonlinear rollout passes the checks. It competes with admissible sampled plans, and the actual source is recorded as `officialsample` or `LQRbackup`. It is not renamed as an MPC-computed action. If neither is admissible, the experiment returns `NO_ADMISSIBLE_PLAN` without applying a new input; this is simulation rejection, not a real-hardware emergency controller.

In the two verified six-second local perturbation runs (+/-0.02 rad), all 1,200 plan selections chose LQRbackup. The existing controller's stabilizing capability was retained, but no improvement from sampling MPC was demonstrated. Each run's 3,000 applied steps and actual torques were independently reconstructed. The corresponding fallen-state test found no admissible plan among its 64 candidates and stopped before advancing. A finite batch rejection is not a proof that no feasible control trajectory exists.

The independent reader now rejects failed consistency flags, missing/contradictory plan counts, nonfinite predictions, changed tolerances and mismatched time steps instead of treating a replayable trace as proof that every planner gate passed. Source snapshots preserve the identity of older results. Local stabilization and get-up from a fallen state are distinct outcome fields; the first local receipt predates that label correction, and the later admission report classifies it from its actual initial state rather than claiming a get-up.

The next planning problem is to represent and initialize useful feasible input trajectories. A separate diagnostic sampled the known LQR motor trajectory into the same piecewise-linear family: 8–36 evenly spaced knots over 1 s did not retain local posture in open-loop replay, while denser samples over 0.2 s approximated the short transient much more closely. This isolates input representation from motor authority, but is not a new sampling-MPC success claim. No hardware dimensions, torque limits, gains, added impact load or task thresholds were changed to obtain these observations.
