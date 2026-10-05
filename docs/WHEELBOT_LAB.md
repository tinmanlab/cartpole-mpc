# Wheelbot lab

[Open wheelbot simulation](https://tinmanlab.github.io/cartpole-mpc/wheelbot.html) · [CartPole lab](../index.html)

## Start with the compact live robot

The default robot is a full-contact, crouched planar wheelbot. It begins with its own LQR and Kalman filter running. Use **−3 cm**, **Center**, **+3 cm**, or the goal selector to change the position target without resetting the robot or estimator. Pause, single-step and reset are beside the viewport. Opening advanced settings does not push the robot below the page.

The compact base is an actual ellipsoid, 170 × 140 × 150 mm, not a rounded drawing of a rectangular collider. Its 1 kg lumped body/drive mass has a recomputed ellipsoid inertia, and its COM is 20 mm above the hip axis. The body is shown with an optional COM marker and actual hip/knee locations. The initial hip is 0.55 rad and knee is −1.10 rad, approximately 63° knee flexion; the equilibrium height, body pitch and holding torques are solved from this model. This is an explicit design assumption, not a claim that every motor belongs at the COM or that remote knee actuation has been implemented.

The controller is newly designed for the compact model. It receives only KF estimates. Measurements are five noisy position/orientation channels plus an explicitly simulated wheel-encoder angular-rate channel with 0.02 rad/s standard deviation. The encoder-rate channel is a teaching sensor model, not calibrated hardware data; the old reference experiments retain their original five-channel observer. The new steady-state Kalman filter and LQR matrices are recomputed from the actual five-substep MuJoCo transition. Tight physical, velocity and tracking acceptance is checked independently. The initial five-channel compact design did balance, but wheel-rate jitter failed the motion criterion; this failure was not re-labelled as success.

`assets/wheelbot/live_model.xml`, `live_profile.json` and `live_design.json` own this model and its declared test scope. `python scripts/design_wheelbot_live.py` regenerates the model, trim and profile with the pinned native environment. `python tests/test_wheelbot_live_model.py` independently verifies them and generates 20 native ten-second trials. `node tests/test_wheelbot_live_parity.mjs` compares 20,000 actual WASM control steps against those native trials. Neither imported gains nor old benchmark certificates are applied to the new robot.

Control and physics are fixed-step; canvas rendering and text/plots are scheduled separately. Geometry is drawn once per frame, expensive outcome summaries are evaluated on request or completion, model metadata is cached, tables update only when open, and plots use bounded histories. The displayed real-time factor uses simulated time divided by measured wall time. It is not a label copied from the intended frequency. Browser pacing tests and their screenshots are recorded by CI; observed performance is not a hard-real-time or hardware guarantee.

## Reference experiments

Select **Reference · standing / jump** to use the original wheel-only-contact benchmark and its matching local MPC/KF, planned jump or pulse profiles. Expand **Reference experiments** for these controls. This model is deliberately distinct from the new compact body: a profile does not become valid merely because the new robot looks similar.

**Try contact lift · apply reference model**, followed by **Reset & lift/hold · left/right**, loads the preserved full-contact reference model and runs 250 commands at 2 ms. It performs approximately 20 mm of hip-edge lift and hold in 0.5 seconds, not full get-up. This particular tracker uses an offline trajectory and exact simulator state; it is not a new online MPC or sensor-based contact controller.

The advanced **Robot design** editor still describes the reference primitive geometry. Its mass/link/motor edits are applied only by **Apply physical model**; changing form fields alone does not change active physics. Applying a different XML invalidates incompatible standing, jump and lift profiles. The editor is not a universal importer or automatic controller tuner. **Restore verified benchmark** returns to the reference model; select **Compact · crouched balance** to return to the new live default.

## Provenance and boundaries

Selected leg lengths, tire dimensions and effort limits are derived from the pinned Upkie description. The [source record](../assets/wheelbot/source/provenance.json), [original URDF](../assets/wheelbot/source/upkie.urdf) and [Apache-2.0 license](../assets/wheelbot/source/LICENSE) are retained. The compact torso is a new declared primitive/lumped-drive assumption. Motor assemblies, transmission compliance, current/thermal limits and real sensors are not independently identified. There is no 3D one-wheel balance or physical-hardware validation.

Reference tracking checks use only committed model/profile/protocol files: `python tests/export_wheelbot_tracking_reference.py`, then `node tests/test_wheelbot_contact_tracking_wasm.mjs`. Historical trace digests are provenance, not dependencies on an unavailable local file. Existing standing, jump, pulse, contact and malformed-profile browser regressions remain part of CI alongside the compact model and real-time pacing checks.

MuJoCo is the reusable physics engine. This lab provides explicit model-specific adapters, not arbitrary-robot compatibility or a registered WebMCP interface. Other structures require validated state/actuator/sensor mappings and new controller design. Full get-up and arbitrary-fall recovery remain outside the published claim.
