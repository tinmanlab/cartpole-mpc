# Sim2real correctness review — 2026-10-04

> Historical first audit at `fc7a8e7`: its defects and numerical failures are retained as original evidence. The follow-up [MuJoCo WASM runtime](MUJOCO_WASM_RUNTIME.md) replaces the browser physics and QP optimizer and closes the strict parity gap; use its current receipts for runtime status.

## Decision

Keep CartPole as a **control/estimation commissioning lab**, not a catalogue of increasingly sophisticated acronyms. Promote a component only when its model, information, numerical, timing, and failure contracts are explicit and tested. Reuse native libraries as numerical authorities; retain browser implementations as inspectable teaching approximations where appropriate.

This review distinguishes three sources:

- **User-supplied research document:** proposes identification, covariance calibration, controller/co-tuning, LTV/NMPC/MHE, independent train/validation/test, and native OCS2/acados/Pinocchio authority. Its final ordering puts identification after tuning, although its earlier rationale correctly puts identification first.
- **Inspected repository:** main `9e5b2049f460b427a4d9233ef4ff002c68490625`, plus the existing uncommitted `feat/advanced-sim2real-failures` work. LTV, MHE, identification and tuning are already present; this is not a request to add them again. The inherited changes were copied into the isolated `audit/sim2real-native-authority` worktree without changing the original working tree.
- **New evidence:** contract regressions and direct numerical comparisons with SciPy, OSQP and MuJoCo. References below support methods; they do not establish this repository's hardware readiness.

Snapshot SHA-256 of the inherited patch: `f828932563de828a7abe226907b6395d2e601321f7fe0f3d83679b0fa3db5fa3`.

## 1. Reproduced implementation defects and corrections

`tests/test_commissioning_contracts.js` failed all eight checks before correction and passed all eight after correction. Existing `npm run check` had passed before these tests existed. Therefore a passing old test suite was not sufficient evidence of these contracts.

| Area | Before | Correction / regression |
|---|---|---|
| UKF process noise | Q was added to predicted state covariance but omitted from the measurement covariance and cross-covariance. | For the actual locally linear sensor `h(x)=[x,theta]`, project the full predicted P: `S=H P H^T+R`, `P_xz=P H^T`. Retain sigma-point nonlinear prediction and Joseph-form posterior covariance. Changing Q by diag(0.01,0.02,0.03,0.04) now changes the two S diagonals by 0.01 and 0.03 rather than zero. This is not a generic nonlinear measurement UKF. |
| Missing measurements | The plant marked a sample stale but the observer repeatedly assimilated its held value. | Pass a null measurement on known transport dropout; prediction only, no innovation covariance, no neural correction using unavailable inputs. The measured value remains in the trace as transport evidence, with `measurementUsed=false`. |
| Raw velocity across gaps | Finite differences assumed every sample was 20 ms apart. | Divide by elapsed sample intervals. This does not infer arbitrary packet timestamps or compensate out-of-sequence measurements. |
| Consistency statistics | `mean(...) || expected_mean` converted both absent samples and genuine zero errors into ideal NIS/NEES means and zero penalty. | Missing data is unavailable/null, zero remains zero, and calibration rejects unavailable statistics rather than rewarding them. |
| MHE arrival covariance | Only the diagonal of the EKF arrival covariance was retained. Angle arrival errors were not wrapped. | Whiten using the full Cholesky factor and wrapped angular error. The posterior arrival prior excludes the first measurement from subsequent measurement residuals; do not introduce double counting. |
| Track safety trigger | The supervisor tested cart position relative to the goal instead of the fixed physical rail. | Test absolute world cart position. This repairs the coordinate mistake but does not certify the backup policy. |

The independent DARE comparison additionally exposed termination at the educational routine's 500-iteration cap. The routine now reports convergence/iteration count, allows the existing fixed-point iteration to finish, and computes K at the returned P. SciPy gain error changed from approximately `1.86e-7` to `1.69e-11`; the predeclared `1e-7` comparison tolerance was not relaxed. This is a numerical bookkeeping correction, not evidence that the old gain was physically unsafe.

The final contract suite also checks DARE termination metadata and explicit covariance provenance (10 contract checks total). Browser regression tests verify real MHE iteration/cost diagnostics and show unavailable innovation as an em dash rather than zero. The episode and live payloads now label simulator-derived R as `injected-noise-oracle`; a provided R is labelled `provided`. This makes the remaining oracle dependency visible, but does not replace it with measured calibration.

The diagnosis previously marked `D08_REALTIME=PASS` from solver-call timing alone. Its regression was reproduced and corrected: the status is now `MEASURED_ONLY` (or investigation/unavailable), with `endToEndRealtimeVerified=false`. This prevents a solver timing probe from being presented as a sensor-to-actuator deadline guarantee.

## 2. Native authority lane actually added

`tests/test_native_reference.py` calls installed libraries rather than introducing a new optimizer or rigid-body dynamics engine. Tested optional dependency pins are in `requirements-native.txt` (NumPy 2.2.6, SciPy 1.15.3, OSQP 1.0.4, MuJoCo 3.3.7; these are tested versions, not claims of the latest versions).

**SciPy:** solve the discrete algebraic Riccati equation from the actual exported A/B/Q/R, rather than only comparing with rounded constants. Obtain chi-square pointwise intervals through `scipy.stats.chi2`.

**OSQP:** assemble the standard sparse state/input QP with variables `x_0..x_N,u_0..u_(N-1)`. Check the factor of two in the objective, equalities, input bounds, and physical rail coordinates. Accept an action only after solved status, finite values, primal/dual residual and independent constraint-violation checks. A non-finite initial state and an initially infeasible hard-rail state both return no action. `action=null` is a rejection signal, NOT a recommendation to command zero force on arbitrary hardware.

**MuJoCo:** build the same ideal cart and uniform rod independently using joints, mass, COM and inertia. Here `l` is COM distance and the complete rod length is `2*l`; COM transverse inertia is `mp*l^2/3`. Positive hinge rotation, upright zero and state order are explicit. Thirty-six forward-acceleration comparisons over three parameter sets agree to approximately `2.23e-14`. A 40-step replay compares JS semi-implicit integration at 1/4/16/64 substeps with a refined MuJoCo RK4 reference. Error decreases with refinement. Different discrete integration maps are not expected to match exactly.

**Boundary:** these are offline numerical reference experiments. They do not replace the live browser solver, implement a hardware bridge, validate the BAM motor port, or establish humanoid contact dynamics.

Receipts: `evidence/native_reference_before.json` and `evidence/native_reference.json`. The latter includes dependency versions, relevant source hashes, fixture hash, tolerances, individual outcomes and `hardware_admission=NOT_EVALUATED`.

### Do not conceal the remaining MPC approximation

For four fixed input-box QPs, the browser plan's relative cost gap is about `6.3e-9` to `6.6e-6`, but its first-action gap is about `0.00050` to `0.05226 N`. Three of four cases fail the predeclared `0.001 N` first-action parity tolerance. This tolerance is a strict numerical-comparison contract, **not a hardware safety limit**. Failure does not prove instability; low cost also does not prove optimizer equivalence.

The two gates deliberately answer different questions:

- default native test: are the independent model/reference/constraint contracts functioning?
- `--require-browser-parity`: does the browser solver meet the declared numerical parity tolerance?

Do not replace the second gate with a weaker tolerance to obtain a green badge. Either keep the teaching controller explicitly approximate, or connect the native solver and compare the complete closed loop under an agreed application-level tolerance.

### Keep problem formulation, discretization and solver names separate

Fixed LTI dynamics, trajectory-local LTV approximations and a nonlinear OCP are different modeling choices. Multiple shooting is a transcription, SQP is a solver method, and SQP-RTI includes a real-time iteration/preparation-feedback execution scheme. They are not interchangeable entries in one solver list. The current LTV teaching bridge uses a one-step iLQR-style local update; it is not a reproduced acados constrained SQP-RTI implementation. Statements about repeated linearization apply to the relevant derivative-based solver family, not every conceivable nonlinear optimizer. Larger state dimension alone does not prove that every single-shooting method is unusable; sparsity, conditioning, contacts, solver design and the measured deadline determine the appropriate native approach.

## 3. Essential sim2real requirements, in dependency order

### A. Signal, coordinate and time contracts — before another algorithm

Declare units, order, sign, reference frame, state manifold, `l` versus full length, input meaning and covariance ownership. Distinguish requested command, transport-delayed command, actuator realization and measured torque/current. Unknown actual force is not an available sensor just because a simulator can reveal it.

Record acquisition time, arrival time, fusion time, control solve start/end, intended application time, actual application time, sequence ID and validity. A dropout is not a fresh copy of the last value. A frozen but plausible sensor is a different fault. Out-of-sequence fusion needs a defined reject/replay/fixed-lag policy. The current null-measurement correction addresses known freshness loss, not arbitrary asynchronous sensing.

Acceptance: coordinate transforms and one-step time alignment pass executable checks; missingness remains missingness throughout exported data; injected latency and model mismatch remain distinguishable.

### B. Model identification and observability — before tuning away errors

Use data that the deployed robot can actually measure. The existing system-ID data uses true states, actuator ID uses realized force, and default estimator R is derived from the simulator's injected noise variance. These are useful controlled experiments, but they are privileged simulation knowledge, not sensor-only hardware identification/calibration.

Keep an explicit information-provenance manifest: `simulation_truth`, `injected_noise_oracle`, `measured`, `identified`, `assumed`, or `unavailable`. A future deployment lane should reject an undeclared oracle dependency.

Check excitation, parameter scaling, sensitivity singular values and parameter correlations. Fit physically admissible mass/inertia/COM/actuator parameters. Evaluate multi-step prediction on separate trajectories, not just one-step fitting on neighboring samples. Separate actuator gain, delay, friction and bias experiments where the available measurements cannot jointly identify them.

For a floating-base robot, local CartPole rank-4 observability does not prove global position/yaw observability or contact reliability. Define error-state geometry and gauge freedoms. Pinocchio is a candidate dynamics/derivative authority, not an excuse to copy its algorithms into this browser engine.

### C. Estimator calibration and consistency

Separate continuous-time noise spectral density from discrete Q; in general `Q_d = integral exp(A*t) G Q_c G^T exp(A^T*t) dt`, not an unqualified constant copied across sample rates. Use Q_e/R_e/P0 versus Q_c/R_c/Q_f naming consistently.

NIS tests prediction innovations; NEES additionally requires suitable ground truth and the correct error-space dimension. Marginal chi-square bounds assume the stated Gaussian model. Do not count correlated time steps as independent Monte Carlo runs or infer certainty from a pooled mean alone. Report missing/invalid covariance, innovation whiteness, bias, uncertainty coverage and calibration exclusions. Online R adaptation cannot make an unobservable state observable or identify the cause of every innovation spike.

MHE is valuable when constraints, delayed measurements or windowed inference justify it. It is not automatically superior or obligatory. The current deterministic shooting MHE omits explicit process-noise decision variables and has no calibrated output covariance; it remains a teaching approximation. A production constrained MHE should be reproduced from a native upstream example with arrival-cost and process-noise conventions checked.

### D. Controller feasibility, numerical health and execution

A large quadratic rail penalty is not a hard rail bound. Conversely, input-constrained or soft-constrained MPC is still MPC; do not redefine MPC to require every state constraint to be hard. Choose hard constraints, slack variables and penalties deliberately and report their violations separately.

Monitor solver status, primal/dual/KKT residuals where available, feasibility, conditioning/scaling, warm-start validity and iteration termination. An accepted line-search step is not proof of convergence. For recursive feasibility/stability claims specify the terminal ingredients, feasible domain, model uncertainty assumptions and fallback domain actually used. Finite scenarios do not establish a tube/chance-constrained guarantee.

Measure **end-to-end** sensor-to-actuator age, not just mean solver time. Reject stale plans. Stress overruns, missed packets, invalid states, solver failure, saturated/rate-limited commands, state-dependent torque-speed limits and history-dependent thermal limits. The existing heuristic thermal/torque-speed scenarios are fault probes, not identified actuator models.

A saturated LQR backup is not automatically safe. A deployment supervisor needs a tested recovery region, physical limits, watchdog and platform-specific protective stop/energy policy. Software tests do not replace hardware protections.

### E. Tuning and release admission

First freeze a functioning identified/calibrated/native baseline; then tune normalized cost parameters for the actual task. Smooth implicit/differentiable tuning is appropriate only where solver differentiation and convergence assumptions hold. Structural/discrete choices belong outside that derivative path. Safe BO is an optional, assumption-dependent hardware exploration method, not a replacement for a safe seed and independent protective layers.

Freeze train/validation/test by trajectory, conditions, seeds and model regions. Keep a separate final test that is not repeatedly used to guide subsequent design; the current acceptance logic includes the test score and becomes test-set leakage if repeatedly optimized against. Maintain trial/config/code manifests. Report failure counts and uncertainty, not only RMSE. Under independent Bernoulli assumptions, zero failures in n tests only gives a one-sided 95% upper bound `1-0.05^(1/n)`; small seed sets cannot support rare-failure guarantees.

Advance through software-in-loop, target-processor-in-loop, hardware-in-loop and bounded hardware commissioning with explicit admission evidence at each stage. CartPole lacks free-floating base/contact switching; transfer the contracts and methods, not a claim that its successful controller automatically handles humanoids.

## 4. What to implement next, and what not to build

**Next focused deliverable:** native constrained MPC + calibrated estimator replay on independently simulated/measured data, with timestamps, provenance, status/residuals and a reject/fallback contract. Reuse the OSQP reference transcription; reproduce an official acados pendulum NMPC/MHE example before implementing an adapter. Add actual joint/contact dynamics only in the separate robot-scale integration.

Do not add more observer/controller pages to compensate for the missing execution contract. Do not simultaneously introduce several production solver frameworks. Use OSQP for the convex reference; choose acados or the robot's existing OCS2 stack for the nonlinear/switched path on the basis of a reproduced upstream example. Use Pinocchio for native rigid-body calculations where applicable. The local contribution is model/data adapters, commissioning experiments, diagnostics and evidence—not another handwritten SQP, rigid-body engine, contact-aided InEKF or covariance optimizer.

The attached document's `ContEst` arXiv reference (`2609.36090`) was not successfully retrieved in this review. It is unverified here, not declared false. Do not make it an essential implementation dependency. DiffTune-MPC and LegBiCal are research references, not components reproduced by this patch; no conference-acceptance claim is made for LegBiCal.

## 5. Reproduce

```sh
npm run check
python3 tests/test_browser.py
python3 -m venv .venv-native
.venv-native/bin/python -m pip install -r requirements-native.txt
.venv-native/bin/python tests/test_native_reference.py
.venv-native/bin/python tests/test_native_reference.py --require-browser-parity
npm run commission
```

The strict parity command currently returns nonzero by design because the measured browser approximation is outside the unchanged comparison tolerance. This is a reported remaining boundary, not a silently skipped test. Keep its receipt alongside passing reference checks. No hardware admission or main/Pages deployment is implied by these commands.

## Primary references

- OSQP official MPC example: https://osqp.org/docs/examples/mpc.html
- OSQP convergence and infeasibility: https://osqp.org/docs/solver/index.html
- SciPy DARE: https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_discrete_are.html
- SciPy chi-square distribution: https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.chi2.html
- MuJoCo dynamics/integration: https://mujoco.readthedocs.io/en/stable/computation/index.html
- acados supported features, RTI, constraints, MHE, sensitivities: https://docs.acados.org/features/
- OCS2 overview: https://leggedrobotics.github.io/ocs2/overview.html
- FilterPy recursive-filter reference: https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html
- DiffTune-MPC: https://arxiv.org/abs/2312.11384
- LegBiCal covariance/kinematic calibration: https://arxiv.org/abs/2510.11539
- Safe legged-controller tuning: https://proceedings.mlr.press/v229/widmer23a.html

Library code is imported, not vendored into this change. The QP adapter follows the formulation explained by the official OSQP example; no claim of upstream authorship or endorsement is made. Existing plant/BAM attribution remains intact.
