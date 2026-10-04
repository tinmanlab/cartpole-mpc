# Third-party notices

## Rhoban/BAM actuator equations

src/plant.js contains a JavaScript adaptation of selected actuator/friction equations from Rhoban/BAM.

- Project: https://github.com/Rhoban/bam
- License: Apache License 2.0
- License text: licenses/Apache-2.0.txt

The CartPole coupling, educational scenarios, local controller/observer adapters, small iLQR-style routines and UI are local. The main QP numerical solver is upstream quadprog, and physics is upstream MuJoCo WASM, as attributed below; not all numerical solvers are locally authored.

## Figure / wb_humanoid_mpc

The centroidal and full-order NMPC teaching pages were derived from architecture/formulation review of the pinned external comparator used by tinmanlab/figure:

- manumerous/wb_humanoid_mpc at ab5dbfd1df07a33258ce7f2998f5c315f1fcc8b9

No OCS2, Pinocchio, or wb_humanoid_mpc source code is copied into this repository. The CartPole implementations are local educational analogues and are explicitly labeled as such.

## Related tinmanlab teaching projects

Visual and documentation patterns were informed by:

- https://github.com/tinmanlab/cartpole-sonic
- https://github.com/tinmanlab/cartpole_PPO
- https://github.com/tinmanlab/cartpole-transformer
- https://github.com/tinmanlab/cartpole-diffusion-v02

The frozen PPO actor artifact is reused from the related CartPole training project. This is not a newly trained policy or a reproduced paper benchmark. Other source reuse is identified by the specific lineage/notices in this file.

## Official MuJoCo WebAssembly runtime

- Upstream: https://github.com/google-deepmind/mujoco
- npm package `@mujoco/mujoco`, tested pin **3.7.0**; runtime version is checked independently.
- Unmodified JavaScript loader, WASM binary and upstream README are shipped under `vendor/mujoco/`.
- License: Apache-2.0; full license shipped as `vendor/mujoco/LICENSE`.
- The local `assets/cartpole.xml` and state/parameter adapter are this repository's work, not an upstream hardware/model reproduction.

## quadprog browser/Node QP solver

- Upstream: https://github.com/albertosantini/quadprog
- npm package `quadprog`, tested pin **1.6.1**, retaining its Goldfarb–Idnani implementation and one-indexed API.
- Upstream module sources are unmodified; a local CommonJS packaging wrapper exposes them in the browser.
- License supplied upstream: MIT; full notice and README are shipped under `vendor/quadprog/`.
- The local MPC transcription and residual checks are not claimed to be upstream code. OSQP is used separately as an independent numerical reference.

`vendor/manifest.json`, `package-lock.json` and `scripts/vendor_runtime.py --check` provide exact artifact/source identity. No upstream endorsement or hardware safety certification is implied.

## Serial N-pendulum model construction

The N-link generator follows the nested-body construction pattern in google-deepmind/dm_control, `dm_control/suite/cartpole.py::_make_model`, inspected at commit `87e046bfeab1d6c1ffb40f9ee2a7459a38778c74` (The dm_control Authors, Apache-2.0). Our generator extends this repository's explicit-inertia uniform-rod MJCF instead of importing the dm_control model/reward/policy. Existing Apache-2.0 license text is retained in `licenses/Apache-2.0.txt`. No upstream benchmark performance or endorsement is claimed. MuJoCo supplies dynamics; SciPy supplies DARE/Kalman design; quadprog and OSQP supply the actual QP solves.

The pre-stabilized N-chain path changes the QP transcription via standard LQR feedback coordinates and still uses the same pinned quadprog solver. An independent NumPy/OSQP transcription checks it. It does not claim to implement a new optimizer or reproduce the specific Toeplitz preconditioner in McInerney et al. (arXiv:2010.08572). The N-dimensional EKF reuses the local matrix routines and official MuJoCo transition/Jacobian interface; Lyapunov/noise diagnostics use SciPy. No new third-party runtime or binary is introduced.

## Optional sequential tuning

The optional offline lane imports SMAC3 2.4.1, ConfigSpace 1.2.1, scikit-learn 1.7.2 and SciPy. SMAC owns the random-forest/EI search, Sobol initial design and standard Intensifier/racing. The local contribution is the bounded configuration mapping, existing task-evaluator connection, explicit failure-order encoding, comparison protocol and verification. Upstream packages and their license notices remain in the isolated optional environment; no Python optimizer or model binary is bundled into the browser. This is neither a new optimizer implementation nor upstream endorsement. See `docs/SEQUENTIAL_TUNING.md` and `requirements-tuning.txt`.

## Wheelbot source lineage

`assets/wheelbot/source/upkie.urdf` and `LICENSE` copy official `upkie/upkie_description` at commit `94735fbe6137276a41de0ff4cc04d2e533fa9e33` under Apache-2.0. Source: https://github.com/upkie/upkie_description/tree/94735fbe6137276a41de0ff4cc04d2e533fa9e33 . The generated primitive-only sagittal model is a modified, simplified single-side derivative. No upstream meshes (including the separately CC BY 4.0 wheel tire) are included or loaded. See `docs/WHEELBOT_2D.md` and generated provenance for exact parameter lineage.
