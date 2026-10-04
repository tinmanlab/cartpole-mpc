# MuJoCo WASM runtime and asset contract

## What executes

The public browser lab loads the official `@mujoco/mujoco` **3.7.0** WebAssembly runtime. The package version, `mj_versionString()` and Python MuJoCo reference version are checked separately. Startup fails visibly if the binary or MJCF cannot load; there is no silent JavaScript physics fallback. The checked-in single-threaded runtime works on the existing static GitHub Pages site without introducing a server, CDN dependency or cross-origin-isolation requirement.

`src/mujoco_backend.mjs` maps `[x, v, theta, omega]` to MuJoCo's `[x, theta]` positions and `[v, omega]` velocities. Plant propagation, nonlinear estimator prediction and nonlinear controller rollout all call this backend after browser initialization. The nominal predictive model and hidden-mismatch plant use the same asset family with explicitly different parameters; the predictor is not given the hidden true mass/friction values.

The existing JavaScript equations remain a labelled **differential reference for legacy/unit tests**, not the browser runtime. The browser test replaces `Plant.integrate` with a throwing function and still exercises simulation and estimation. `npm run commission` initializes the WASM backend too and records engine version, asset hash, numerical source hashes and whether those sources stayed unchanged during the experiment.

## One canonical asset

`assets/cartpole.xml` is a locally authored MJCF matching this lab's uniform-rod model. It is not a renamed Gymnasium or dm_control asset and is not claimed to reproduce a manufactured cart, servo, or humanoid.

| Quantity | Contract |
|---|---|
| Coordinates | SI; world +x is cart travel; hinge +y tips the pole toward +x |
| State | `[x (m), v (m/s), theta (rad), omega (rad/s)]`; theta=0 upright |
| Nominal cart / pole mass | 1 kg / 0.1 kg |
| Nominal pole COM distance | l=0.5 m; endpoint separation 2l=1.0 m |
| Pole transverse COM inertia | mp*l*l/3; not point-mass inertia |
| Actuation | Ideal scalar horizontal force in N, motor gear=1 |
| Discretization | MuJoCo Euler at 0.005 s; four steps per 0.02 s control interval |
| Rail | World-position boundary ±2.4 m; not a hidden collision joint stop |
| Geometry | Explicit non-colliding rail/cart/pole geometry; visual thickness does not determine inertia |
| Deliberately absent | Wheel rolling/slip, contact forces, free-floating base, identified motor electronics, robot hardware |

The model's masses, COM, inertia, rod visual geometry and endpoint sites are changed together for parameterized model-mismatch experiments, followed by `mj_setConst`. The Canvas viewer draws MuJoCo forward-kinematics sites and compiled geometry dimensions; it no longer invents wheel circles, cart dimensions or a separate rod length. The view is a **2D projection of MuJoCo geometry**, not the native MuJoCo OpenGL renderer. Truth geometry uses plant parameters; estimate/horizon overlays use the nominal model deliberately.

This state-only adapter resets scratch MuJoCo data for each transition. That is valid for the current two-DOF ideal-force model, with all additional actuator/transport state explicitly held by `LabPlant`. It is not a generic adapter for robots with contact warm starts, activation state, plugins or other MuJoCo history. A larger robot needs a declared full state and persistent runtime data; do not copy this assumption unchanged.

## Runtime identity and upstream packaging

`package-lock.json` pins the npm artifacts. `scripts/vendor_runtime.py` copies unmodified official runtime files and wraps unmodified `quadprog@1.6.1` CommonJS modules for browser use. `vendor/manifest.json` records upstream package versions, exact file sizes, SHA-256 values and original quadprog source hashes. `npm run check:assets` compares the published files byte-for-byte with the pinned installed packages. Upstream licenses are shipped beside both runtimes.

During verification, the installed npm `@mujoco/mujoco@3.6.0` binary returned runtime string `3.7.0`. The version guard rejected that combination. The admitted combination is npm 3.7.0 / WASM 3.7.0 / Python 3.7.0, verified by execution. This records an observed packaging mismatch, not a claim about its cause or about all upstream distributions.

## MPC solver: what changed and what did not

Linear MPC and the reduced outer planner call the pinned **quadprog Goldfarb–Idnani QP solver**. `src/qp.js` owns only the finite-horizon transcription and acceptance checks, not an optimizer implementation. The same packaged solver runs in the browser and Node. It is JavaScript upstream code; **OSQP is the independent native reference, not the browser solver**.

The added `hard_mpc` selection imposes input bounds and world-coordinate position constraints at every predicted stage. The existing `state_mpc` remains a separately labelled soft-penalty teaching mode. A small cost or accepted line-search step is not used as proof of numerical convergence. QP acceptance checks finite values, primal feasibility, dual sign, complementarity and stationarity residuals. Infeasible/invalid problems are rejected.

The previous fixed four-case 0.001 N first-action parity test against OSQP is retained unchanged and now passes. An additional active-rail test compares both cost and first action with OSQP. Browser tests repeat the comparison using the packaged browser solver rather than relying solely on Node `require()` results.

Hard constraints constrain the **prediction using the estimated state and nominal model**. They do not guarantee the true noisy/mismatched plant remains inside the rail. Recursive feasibility, robust invariant sets and a certified recovery policy are not claimed. NMPC/LTV and shooting MHE retain the explicit educational implementation boundaries; acados/OCS2 production reproductions are not added by this change.

## Failure and timing contract

A non-finite command or solver exception stops the simulation before applying that invalid input. A failed plant envelope stops further stepping. The UI shows FAULT and requires reset; it does not label a zero-force guess as a safe hardware policy. Missing runtime assets keep controls disabled rather than switching engines.

Live state and trace expose source/post-step simulation times, solve-to-application compute time, total synchronous step compute time and missed compute deadlines. These are browser-host measurements in a synchronously stepped simulation. Sensor transport to physical hardware, arbitrary out-of-order sensor fusion, stale hardware commands and hard-real-time scheduling remain unverified. `D08_REALTIME` distinguishes measured-only/investigation from end-to-end admission; consult the exact receipt rather than treating this document as live status.

The default hard-rail controller also has a separate mixed/latency/sim2real/glitch stress grid in `evidence/mujoco_wasm.json`. Completed trials, state-envelope failures and infeasible-QP rejections are distinct outcomes. The grid is not required to pretend that all disturbances are recoverable. The comparison UI preserves each rejected solver as a `REJECT` cell and yields between cells; one failed candidate no longer discards the entire comparison.

The linear-MPC lesson and controller documentation now name the actual upstream quadprog solver. The previous projected-gradient/backtracking description and unsupported warm-start claim were removed from that lesson. These corrections are guarded by a public-description regression.

## Verification receipts

- `evidence/native_reference.json`: SciPy/OSQP numerical comparisons, including the unchanged strict browser-QP tolerance and an active hard-rail case.
- `evidence/mujoco_wasm.json`: actual WASM dynamics, parameter/geometry checks, closed-loop combinations and a fixed input/state replay.
- `evidence/wasm_native_parity.json`: Python MuJoCo loads the exact same checked-in MJCF and replays the WASM inputs.
- `evidence/browser_wasm.json`: real HTTP WASM/MJCF requests, no legacy fallback, unshimmed UI, QP parity, hard constraints and failure paths.
- `evidence/commissioning.json`: regenerated WASM commissioning results and information/source provenance.

The first page of the browser integration test explicitly supplies a WebMCP registration shim to test callbacks. A second page runs the normal UI with no shim and reports actual native WebMCP availability separately. Neither test fabricates browser-native WebMCP support.

## Reproduce

```sh
npm ci
npm run check
npm run test:wasm
python3 -m venv .venv-native
.venv-native/bin/python -m pip install -r requirements-native.txt
npm run test:native:parity
npm run test:wasm-native
python3 -m pip install playwright==1.61.0
python3 -m playwright install chromium
npm run test:browser
npm run commission
```

Serve the repository through HTTP (`python3 serve.py`), not `file://`. The published directory must include `assets/cartpole.xml`, `src/mujoco_backend.mjs` and `vendor/`; `index.html` is no longer a dependency-free single-file simulation. The existing CI workflow exercises source/asset checks, actual WASM, strict native parity and real-browser failure tests.

Hardware admission remains **NOT_EVALUATED**. Estimator noise/model calibration still uses disclosed simulated information; a real sensor-only calibration dataset and target hardware were not available in this task. Successful CartPole tests do not establish humanoid control performance.

## Primary upstream references

- Official MuJoCo WASM source and usage: https://github.com/google-deepmind/mujoco/tree/3.7.0/wasm
- MuJoCo model/integration conventions: https://mujoco.readthedocs.io/en/stable/computation/index.html
- quadprog upstream: https://github.com/albertosantini/quadprog
- Independent OSQP MPC formulation: https://osqp.org/docs/examples/mpc.html

The reduced CoM controller now renders only its real reduced prediction. Supervisor overlays label the primary plan when a backup is applied. The educational UI separates units, estimation error and reference tracking; these display/diagnostic corrections do not claim a new plant or controller safety proof.
