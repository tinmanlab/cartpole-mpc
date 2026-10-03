# CartPole MPC

> An interactive control-and-estimation lab for seeing how **PID, LQR, linear MPC, reduced-order centroidal-style MPC, full nonlinear NMPC, PPO, KF, EKF, invariant filtering, and learned estimator corrections** fit together on one shared CartPole plant.

[**Open the live lab →**](https://tinmanlab.github.io/cartpole-mpc/) · [Full NMPC](https://tinmanlab.github.io/cartpole-mpc/#full_nmpc) · [InEKF bridge](https://tinmanlab.github.io/cartpole-mpc/#inekf) · [FOCUS bridge](https://tinmanlab.github.io/cartpole-mpc/#focus)

<p align="center">
  <a href="https://tinmanlab.github.io/cartpole-mpc/">
    <img src="media/cartpole-mpc-demo.gif" alt="CartPole MPC live controller and observer demo" width="100%">
  </a>
</p>

[High-resolution WebM recording](media/cartpole-mpc-demo.webm)

The loop above is a real browser run of the integrated lab: **full nonlinear NMPC + adaptive measurement covariance**, with sensor noise, pushes, live state estimation, solver timing, and horizon visualization. The adaptive covariance path is an educational bridge to CoCo/FOCUS-style reliability handling; it is not a reproduction of those learned humanoid estimators.

## Start here

The entire lab uses one control loop:

    nonlinear plant
          ↓
        sensor
          ↓
       observer
       x_hat, P
          ↓
      controller
          ↓
        force u
          ↓
    nonlinear plant

Change only one block and watch what changes in the same live simulation and graphs.

### Controller path

    PID
     ↓ add model + cost
    LQR
     ↓ add finite horizon
    Linear MPC
     ├─ reduced model → Centroidal-style MPC
     └─ full nonlinear model → Full NMPC

    PPO is the parallel learned-policy path:
    online optimization ↔ offline-trained policy

### Observer path

    raw sensor
       ↓ model uncertainty
    Kalman Filter
       ↓ nonlinear dynamics
    EKF
       ↓ state-space geometry
    InEKF concept

Then learned information can enter at different points:

| Method | What learning changes |
|---|---|
| Lin | contact on/off gate |
| Youm / NMN | learned velocity measurement |
| InNKF | posterior state residual |
| CoCo-InEKF | contact measurement covariance |
| FOCUS | continuous FK measurement reliability |

## What is actually implemented?

| Runtime mode | What runs in this repo |
|---|---|
| PID | live cascaded feedback controller |
| LQR | Riccati state-feedback controller |
| Linear MPC | N=30 finite-horizon receding controller |
| Centroidal-style MPC | N=32 reduced horizontal CoM planner + downstream full-state stabilizer |
| Full nonlinear NMPC | N=30 nonlinear rollout, per-stage Jacobians, backward quadratic solve, bounded line search, warm start |
| PPO | frozen robust actor from the related CartPole PPO lab |
| KF | linear Kalman filter |
| EKF | nonlinear prediction + numerical Jacobian covariance propagation |
| Invariant-error bridge | SO(2) wrapped angle innovation; not the full Hartley humanoid InEKF |
| Learned residual | locally trained residual MLP behind the geometry-aware EKF |
| Adaptive R | online measurement-covariance modulation; heuristic bridge to CoCo/FOCUS |

The distinction between **implementation**, **architecture analogue**, and **paper-only concept** is deliberate. See [Implementation boundaries](docs/IMPLEMENTATION_BOUNDARIES.md).

## Why two MPC pages?

The distinction comes directly from the model-based humanoid stacks reviewed in tinmanlab/figure.

### Centroidal MPC

The pinned humanoid reference optimizes a reduced state:

    state  = centroidal momentum + base pose + joint angles
    input  = contact wrenches + joint velocities

and sends the plan through inverse dynamics / lower-level joint control.

CartPole has no feet or independent contact-wrench variables, so this repo does not invent fake humanoid contacts. Instead it preserves the same hierarchy:

    horizontal CoM reduced model
              ↓
        predictive planner
              ↓
       future CoM reference
              ↓
      full-state stabilizer
              ↓
          CartPole force

### Full nonlinear NMPC

The full-order teaching controller instead puts the **same nonlinear CartPole plant used by the live simulator** inside the prediction horizon.

At every control tick it:

1. warm-starts the previous control sequence,
2. rolls out the nonlinear dynamics,
3. computes stage Jacobians,
4. solves a local quadratic backward pass,
5. runs a bounded forward line search,
6. applies only the first force,
7. shifts the horizon and repeats.

This is a real nonlinear receding-horizon solver. It is not claimed to be a source port of OCS2 multiple-shooting SQP.

## Quick start

### Fastest: GitHub Pages

Open:

**https://tinmanlab.github.io/cartpole-mpc/**

Useful lessons:

- [PID](https://tinmanlab.github.io/cartpole-mpc/#pid)
- [LQR](https://tinmanlab.github.io/cartpole-mpc/#lqr)
- [Linear MPC](https://tinmanlab.github.io/cartpole-mpc/#linear_mpc)
- [Centroidal-style MPC](https://tinmanlab.github.io/cartpole-mpc/#centroidal_mpc)
- [Full nonlinear NMPC](https://tinmanlab.github.io/cartpole-mpc/#full_nmpc)
- [Kalman Filter](https://tinmanlab.github.io/cartpole-mpc/#kf)
- [EKF](https://tinmanlab.github.io/cartpole-mpc/#ekf)
- [InEKF bridge](https://tinmanlab.github.io/cartpole-mpc/#inekf)
- [InNKF](https://tinmanlab.github.io/cartpole-mpc/#innkf)
- [CoCo-InEKF](https://tinmanlab.github.io/cartpole-mpc/#coco)
- [FOCUS](https://tinmanlab.github.io/cartpole-mpc/#focus)

### Run locally

No runtime dependency or build step is required for the checked-in page.

    git clone https://github.com/tinmanlab/cartpole-mpc.git
    cd cartpole-mpc
    python3 serve.py

Open:

    http://localhost:8765

For development:

    npm ci
    npm run check
    python3 scripts/build.py

## A good learning sequence

Do not learn every acronym at once.

1. **PID → LQR**: feedback versus model-derived feedback.
2. **LQR → Linear MPC**: why a horizon and constraints change the problem.
3. **Linear MPC → Centroidal / Full NMPC**: reduced-order versus full nonlinear prediction.
4. **Raw → KF → EKF**: why a state estimator is needed and where P, Q, R, K appear.
5. **EKF → InEKF**: why rotation/pose geometry changes the definition of estimation error.
6. **Lin → Youm → InNKF → CoCo → FOCUS**: where learned information can safely enter a model-based estimator.
7. Combine controller and observer under the same noise, model mismatch, glitch, and push scenarios.

More detail:

- [Controllers](docs/CONTROLLERS.md)
- [Observers and learned estimator lineage](docs/OBSERVERS.md)
- [System architecture](docs/ARCHITECTURE.md)
- [Implementation boundaries](docs/IMPLEMENTATION_BOUNDARIES.md)
- [Validation](docs/VALIDATION.md)

## Fixed evidence

The repo ships deterministic evidence under evidence/.

Representative fixed probes from the current implementation:

- six controller families pass the nominal truth-state probe,
- reduced centroidal-style MPC exposes a 32-step horizon,
- full NMPC exposes a 30-step nonlinear horizon and bounded iterative solve,
- raw finite-difference velocity estimation fails under the fixed high-noise probe where KF remains stable,
- learned/adaptive estimator paths are reported as CartPole results only, not humanoid benchmark claims.

Run:

    npm test

## WebMCP

When the browser exposes the current WebMCP producer surface, the page registers semantic tools for:

- reading current controller/observer/solver state,
- selecting controller, observer, topic, and scenario,
- applying a push,
- stepping or resetting the experiment,
- running deterministic offline controller × observer probes.

The visual UI and WebMCP tools operate the same experiment state. Without WebMCP, the page works normally as an interactive lab.

## Repository structure

    .
    ├── index.html
    ├── src/
    │   ├── engine.js
    │   ├── plant.js
    │   ├── topics.js
    │   ├── app.js
    │   ├── styles.css
    │   └── shell.html
    ├── assets/
    │   ├── ppo_actor.json
    │   └── residual_model.json
    ├── docs/
    ├── evidence/
    ├── media/
    │   ├── cartpole-mpc-demo.gif
    │   └── cartpole-mpc-demo.webm
    ├── scripts/
    │   ├── build.py
    │   └── record_demo.py
    └── tests/

## References and provenance

This project was built by reading the pinned model-based humanoid comparator used in tinmanlab/figure and by reusing the existing tinmanlab CartPole teaching conventions.

Primary architecture reference:

- manumerous/wb_humanoid_mpc at ab5dbfd1df07a33258ce7f2998f5c315f1fcc8b9
- OCS2: https://github.com/leggedrobotics/ocs2

Estimator lineage taught in the UI:

- Hartley et al., Contact-Aided Invariant EKF: https://arxiv.org/abs/1904.09251
- Lin et al., learned contact events: https://proceedings.mlr.press/v164/lin22b.html
- Youm et al., neural measurement network: https://arxiv.org/abs/2402.00366
- InNKF: https://arxiv.org/abs/2503.00344
- CoCo-InEKF: https://arxiv.org/abs/2605.15122
- FOCUS: https://arxiv.org/abs/2609.02222

Related teaching repos:

- https://github.com/tinmanlab/cartpole-sonic
- https://github.com/tinmanlab/cartpole_PPO
- https://github.com/tinmanlab/cartpole-transformer
- https://github.com/tinmanlab/cartpole-diffusion-v02

See [third-party notices](THIRD_PARTY_NOTICES.md).

## Scope

This is a teaching and research prototype, not a production robot controller.

The CartPole versions preserve the **control/estimation structure** needed to understand the algorithms, but they do not reproduce humanoid multi-contact dynamics, hardware state estimation, actuator limits, safety behavior, or OCS2 solver equivalence.
