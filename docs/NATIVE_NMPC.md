# Native NMPC experiment: solver choice is not controller design

## Decision and scope

An optional **acados v0.6.0 + HPIPM** native research lane is implemented and exercised against the existing MuJoCo WASM plant and EKF. The deployed browser remains unchanged: official MuJoCo WASM + the verified upstream quadprog controller. acados has not been compiled to WASM or promoted into the public runtime.

The motivating failures are reproduced, not discarded: `mixed/302` left the rail and `sim2real/303` encountered an inconsistent QP. Changing the numerical solver and model to converged native SQP alone did not resolve them. Replacing the weak diagonal terminal penalty with the full Riccati terminal matrix did resolve these two development cases, with either the existing QP or native NMPC. This distinguishes a formulation improvement from a solver-brand improvement.

The supplied research document recommends native OCS2/acados/Pinocchio authority rather than enlarging handwritten educational solvers. That is a design direction, not evidence that those components were previously installed. This lane reproduces acados and tests its restricted CartPole adapter; it does not reproduce OCS2, a humanoid controller, or production MHE.

## What is actually reused

- acados `v0.6.0`, commit `503364817c872d474ab5bed219c26760ac267769`.
- HPIPM `e3a56c1caddd7f12d125d84f337b9a9e5c186271` and BLASFEO `d6251233923c9b475fe894fb729fb63ab693e301` from its pinned submodules.
- Official `examples/acados_python/getting_started/minimal_example_ocp.py`, executed without modifying its formulation before the local adapter.
- CasADi **3.7.2**, within the series tested by this acados release. An initial installation resolved 3.8.1 and emitted the upstream untested-version warning; final experiments and official reproduction were rerun with 3.7.2. The exploratory `native_nmpc_baseline.json` retains its actual earlier version rather than being relabelled.
- Native `SQP` and `SQP_RTI`, `PARTIAL_CONDENSING_HPIPM`, generalized Gauss-Newton Hessian, partial horizon 10, Release build, GENERIC architecture targets, OpenMP disabled. No optimizer or analytic Jacobian is implemented locally.

The acados checkout, generated sources and shared libraries remain under ignored `test-results/`; they are not added to browser assets. Exact library bytes, source hashes, dependency versions and model checks are recorded in `evidence/native_nmpc.json`.

## Model, information and cost contracts

The local adapter is deliberately restricted to the canonical ideal-force uniform rod in `assets/cartpole.xml`. It checks coordinates, masses, COM, transverse inertia, dimensions and integration convention. It is **not a general MJCF-to-CasADi converter**. Future asset changes require the differential checks again.

The official pendulum example is not silently treated as our physical model. Our symbolic transition translates the existing uniform-rod formulation into CasADi, with four 5 ms semi-implicit steps per 20 ms control sample. The actual plant continues to run in MuJoCo WASM. Across 36 fixed state/input/parameter cases, the maximum transition discrepancy is approximately `6.66e-16`, and the automatically differentiated Jacobian differs from the existing WASM finite-difference reference by approximately `2.10e-11` (tolerances `1e-10` and `1e-7`).

A single persistent Node stdin/stdout process owns the existing `LabPlant`, simulated sensor and EKF. This is a process-local experiment, not a network service or hardware bridge. The native controller is passed **estimate and goal only**. Truth is retained only for evaluation; hidden true mass, gain and delay are not provided to the controller. Measurement covariance remains explicitly labelled `injected-noise-oracle`.

All compared controllers use:

```
N = 30, control dt = 0.02 s, input = +/-10 N, world rail = +/-2.4 m
Q = diag(2, 0.45, 68, 3.5), R = 0.14
original terminal = diag(8, 1.5, 110, 7)
reference x = (0.5 m, 0, 0, 0)
```

acados least-squares factors are matched explicitly: `W=2Q`, `W_u=2R`, `W_e=2P`, and stage/terminal `cost_scaling=1`. This matches the lab's **sum of costs**, not an implicit dt-scaled integral. An independent evaluation of the returned trajectory checks the objective, including off-diagonal terminal terms. Rail constraints include terminal state and remain in world coordinates, not goal-error coordinates.

LTI QP and nonlinear NMPC still solve different dynamics formulations. Same weights and limits are not a claim that their optimization problems or solution times are universally equivalent.

## Terminal design experiment

The second terminal cost is the full `scipy.linalg.solve_discrete_are(A,B,Q,R)` result at the nominal upright discrete linearization. Its cross terms are retained; simply extracting a new diagonal would change this design.

This is a standard local tail-cost approximation, not another hyperparameter search. No stage weights, noise, seeds, force bounds, model mismatch or horizon were changed between a matched terminal pair. The same full matrix is passed to the local existing-QP instance and native NMPC. No production controller source or public default is changed by this experiment.

Three fixed 240-step cases (4.8 simulated seconds) use a +3 N push for ten steps beginning at k=96. These include both previously known failure cases. Eighteen runs comprise three cases x two terminal designs x three solver modes.

| Case | Original QP / original SQP | DARE QP / DARE SQP | Final position, QP original -> DARE |
|---|---|---|---|
| nominal, seed 43 | both complete 240 steps | both complete 240 steps | -0.6535 m -> +0.4282 m |
| mixed, seed 302 | rail-envelope failure at 237 / 236 applied steps | both complete 240 steps | -2.4080 m -> +0.4684 m |
| sim2real, seed 303 | QP rejection / SQP plan rejection at 212 applied steps | both complete 240 steps | -2.3375 m -> +0.4264 m |

The target is +0.5 m. Completing 240 steps only means no tested envelope/solver failure, **not that target tracking is perfect**. Position tracking RMSE and final state are reported separately. The nominal existing-QP tracking RMSE decreases from about 0.6073 m to 0.2392 m over the full run. Comparing RMSE across early-terminated and full-length runs requires care; failure outcomes must not be hidden behind averages.

These are **development cases**. The terminal design was proposed after baseline diagnosis. They are not an untouched final test, a rare-failure estimate, or proof of robustness. A local DARE terminal cost without a verified terminal invariant set does not establish recursive feasibility or global nonlinear stability.

## RTI admission and recovery

A single RTI step can have acados status 0 yet fail the experiment's declared nonlinear defect tolerance. This reflects the distinction between a successful real-time SQP step and a converged nonlinear program, not an upstream solver bug. We do not loosen the original `1e-5` numerical defect tolerance to make RTI look successful. It is an experiment admission policy, not a hardware safety bound or universal RTI requirement.

The optional managed mode first checks the RTI candidate's finite values, initial-state match, input/rail constraints, nonlinear transition defects and exact nonlinear rollout. On rejection it solves **the same current-estimate problem** with SQP. Only a newly admitted candidate is applied; neither the rejected RTI action, a stale plan nor a guessed zero-force command is substituted. Recovered controls seed the next RTI warm start. If both paths reject, the experiment stops with an explicit rejection outcome.

The managed mode completes all three DARE-terminal development cases, but uses SQP recovery on 147, 184 and 180 of 240 steps respectively. Under this strict policy and noisy estimates it is therefore **not yet an attractive fast-path deployment**. First diagnose why the local warm start frequently leaves the admission tolerance; do not advertise the nominal RTI kernel as the whole execution cost.

Native SQP status 4 / QP minimum-step failure in a stress case is not proof that the nonlinear OCP is globally infeasible. It is reported as a rejected numerical attempt, separately from exact convex infeasibility and actual plant-envelope failure.

## Timing: native kernel versus the complete experiment

The result stores native solver time, Python controller/diagnostic time, process round-trip time and complete synchronous iteration time separately. Builds and code generation are reported separately. The baseline's `solverTimeMs` is a Node controller call including its transcription/acceptance work; acados's value is its internal `time_tot`. These scopes are not interchangeable.

For the recorded DARE-terminal cases, native SQP solver p95 is approximately 0.68-0.77 ms, while the complete diagnostic Python/Node iteration p95 is approximately 7.52-8.68 ms. Managed RTI/SQP iterations are slower because recovery is frequent. Existing QP complete-iteration p95 is approximately 1.34-1.67 ms in the same recorded cases. Host scheduling and warm-up matter; this is not an isolated WCET benchmark or proof that one solver is intrinsically fastest.

The next performance frontier is the adapter: batch state/reference updates, minimize Python/C crossings and validate compiled/batched diagnostics before evaluating a deployment path. Do not first add a GPU solver or another transport service to a four-state problem.

## Solver selection under this project's requirements

| Candidate | Appropriate role here | Current decision |
|---|---|---|
| acados + HPIPM | Structured constrained nonlinear OCP, multiple shooting, RTI and future native MHE/sensitivities | Executed in optional research lane; no public-runtime promotion |
| OSQP / generated C | Independently checked convex QP, repeated fixed-structure problems, possible future compiled browser/embedded convex solver | Existing numerical reference retained; no unnecessary browser replacement |
| TinyMPC / conic extensions | Resource-constrained MCU convex MPC when a measured MCU budget becomes the actual requirement | Researched, not installed; not a drop-in solution for nonlinear/hybrid humanoid dynamics |
| Existing quadprog | Small browser convex QP already tested against OSQP | Retained; terminal formulation matters before replacing it again |

There is no project-wide 'SOTA winner' from these tests. Solver choice depends on nonlinear structure, constraints, sparsity, target compute and required accuracy. A modern solver does not supply missing observability, actuator identification or recovery guarantees.

## Reproduce the optional lane

Linux/WSL with existing CMake, C/C++ compiler and Git. No sudo, service, credential or global shell configuration changes are required. Keep any existing source checkout rather than blindly recloning over it.

```sh
ROOT="$PWD"
python3 -m venv --system-site-packages .venv-native
.venv-native/bin/python -m pip install -r requirements-acados.txt

git clone --branch v0.6.0 --depth 1 https://github.com/acados/acados.git test-results/acados-v0.6.0
A="$ROOT/test-results/acados-v0.6.0"
test "$(git -C "$A" rev-parse HEAD)" = 503364817c872d474ab5bed219c26760ac267769
git -C "$A" submodule update --init --depth 1 external/blasfeo external/hpipm
cmake -S "$A" -B "$A/build" -DCMAKE_BUILD_TYPE=Release -DBLASFEO_TARGET=GENERIC -DHPIPM_TARGET=GENERIC -DACADOS_WITH_OPENMP=OFF -DACADOS_EXAMPLES=OFF
cmake --build "$A/build" --parallel 4
cmake --install "$A/build"
.venv-native/bin/python -m pip install -e "$A/interfaces/acados_template"
export ACADOS_SOURCE_DIR="$A"
export LD_LIBRARY_PATH="$A/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MPLBACKEND=Agg

(cd "$A/examples/acados_python/getting_started" && "$ROOT/.venv-native/bin/python" minimal_example_ocp.py)
.venv-native/bin/python tests/test_native_nmpc.py
.venv-native/bin/python scripts/run_native_nmpc.py
npm run check
```

The default browser and its CI do not require acados. Optional native tests require the explicit local setup above. A successful experiment is not merged/deployed controller acceptance. Next admissible integration work is to validate the terminal-cost option on disjoint conditions and longer runs, then review a minimal browser change; native solver promotion additionally requires reducing and measuring execution-boundary overhead.

## Primary sources and provenance

- acados release: https://github.com/acados/acados/releases/tag/v0.6.0
- Official example: https://github.com/acados/acados/blob/503364817c872d474ab5bed219c26760ac267769/examples/acados_python/getting_started/minimal_example_ocp.py
- Features, RTI phases, MHE, partial condensing, sensitivities: https://docs.acados.org/features/
- Installation and architecture targets: https://docs.acados.org/installation/
- OSQP generated C: https://osqp.org/docs/codegen/index.html
- TinyMPC upstream: https://github.com/TinyMPC/TinyMPC
- SciPy DARE: https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_discrete_are.html

Upstream numerical code is imported, not reimplemented or vendored. The formulation API follows official examples; the restricted uniform-rod translation, admission checks and comparison protocol are local contributions. Preserve upstream attribution and distinguish the unchanged official-example reproduction from our model adaptation. Hardware admission remains **NOT_EVALUATED**.
