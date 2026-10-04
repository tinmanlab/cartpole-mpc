# Third-party notices

## Rhoban/BAM actuator equations

src/plant.js contains a JavaScript adaptation of selected actuator/friction equations from Rhoban/BAM.

- Project: https://github.com/Rhoban/bam
- License: Apache License 2.0
- License text: licenses/Apache-2.0.txt

The CartPole coupling, educational scenarios, controller/observer implementations, MPC/NMPC solvers, and UI are local to this repository.

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

Their source is not bundled here except for the CartPole plant lineage noted above.

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
