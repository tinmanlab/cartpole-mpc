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

The only browser feedback mode is local saturated LQR plus stationary linear KF. The filter predicts deviations with A/B and applied torque, then corrects using noisy x/z localization, torso orientation, hip and knee measurements. True velocities are never sensor inputs. External x/z localization is an explicit teaching channel: this is not an IMU-and-encoders-only feasibility claim. A wheel encoder cannot establish absolute x under slip. Passive zero-torque dynamics are available for comparison. The displayed reference is local and the controller is not a swing-up or global recovery policy.

## Explicit algorithm boundary

Browser MIMO MPC: **NOT_YET_SUPPORTED**. `src/engine.js` / `src/qp.js` and existing CartPole algorithms are unchanged. The historical `solveBoxLinearMpc` condenses only the first B column; it must not receive the three-input plant. A focused test demonstrates that boundary and the wheelbot adapter rejects an MPC request visibly. No quadprog/OSQP MIMO parity claim is made. No EKF is implemented or advertised.

## Reproduction and evidence

Use the existing native Python environment (MuJoCo 3.7.0, SciPy 1.15.3). Run the model generator, offline design script, `tests/test_wheelbot_reference.py`, `npm run test:wheelbot`, and `python3 tests/test_wheelbot_browser.py`. Serve the repository over HTTP and open `wheelbot.html`; it remains paused after validation. `npm run check` still verifies the original lab.

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

Measured fixed-case outcomes (three seeds each): local balance completed/passed 3/3; position completed 3/3 and passed 0/3; small push completed 3/3 and passed 0/3; boundary push completed/passed 0/3, with failure at step 105 after four contact-loss samples in each seed. Position convergence is too slow for the fixed 3-second threshold; no tuning search was performed. See `evidence/wheelbot_commands.json` for exact command exits and explicit not-run checks. Browser MPC, EKF, native MIMO MPC probes, closed-loop mass-mismatch campaigns and hardware validation remain unsupported or not run.

Review correction: errors are reported per state/measurement channel with physical units, not as a single mixed-unit score. The native reference reconstructs every task/step/contact/saturation count from raw traces. The 2D camera follows the torso, with world-position ticks; wheel slip uses m/s and all 11 estimated states are displayed. Browser assertions require 20 actual valid physics steps, not merely any partial number below 20.
