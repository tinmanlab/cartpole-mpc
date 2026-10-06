# Wheelbot: direct control

[Open the simulation](https://tinmanlab.github.io/cartpole-mpc/wheelbot.html) · [CartPole](../index.html)

## Three sliders and actions

The default screen contains the robot, three target sliders, and the action row. Plots, observer details, model editing and earlier experiments are inside **분석 · 이전 실험**, closed initially.

Request **horizontal position −1 to +1 m**, **hip-axis height 0.36–0.49 m**, and **body pitch −10° to +10°**. This planar robot does not provide roll/yaw control. Height refers to the torso origin at the hip axis, not its COM. The dashed body is the target; the solid body is the actual simulated robot. Commands change the reference, not the physical state. Reference acceleration and velocity are limited. **목표 복귀** returns the target without resetting the robot or observer. Arrow keys change the horizontal target by 0.1 m.

**점프** uses the same compact robot and its own verified plan. The robot moves under control into the entry posture, then its three motors produce takeoff, flight, landing and settling. No alternate asset, pose teleport or upward external force is used. Pose editing is locked during the action. Six tested reference cases produced about 0.11 m wheel clearance and 0.34 s flight. This is a bounded hop, not an arbitrary jump planner. Invalid optional jump data disables jumping, not ordinary pose control.

**외란** offers a short push, sinusoidal gust or pitch moment, either direction, with three strengths. Straight arrows show force in N; curved arrows show moment in N·m. These are physical inputs, not state changes. Pulses cannot stack. Strong disturbances may cause a fall; their recovery is not guaranteed.

**일어서기** remains unavailable: motor-driven full get-up has not passed this compact model's criteria. Leaving the tested balance domain disengages motors and invalidates the observer while gravity and full-body collision continue. **초기화** explicitly restores the initial state; it is not physical recovery. Smaller recoverable disturbances are corrected automatically by the active balance controller. Existing recovery probes did not establish a valid floor-to-standing path.

## Model and control

The unchanged compact torso is a 170 × 140 × 150 mm ellipsoid, with 1 kg lumped body/drive mass and recomputed inertia. COM is 20 mm above the hip axis. Nominal hip is 0.55 rad and knee −1.10 rad (about 63° flexion). This is a teaching-model assumption, not a calibrated assembly or universal motor-placement rule. Motor ceilings remain ±16 / ±16 / ±1.7 N·m; physics advances at 2 ms and action feedback at 10 ms.

`live_model.xml`, `live_profile.json` and `live_design.json` own the plant and nominal design. `pose_profiles.json` contains 25 independently solved height/pitch trims. Bilinear scheduling interpolates trim, LQR and Kalman matrices; bounded references control transitions. This is empirically tested gain scheduling, not global Lyapunov certification. Range labels describe admitted commands, not every possible input sequence.

Measurements are five noisy pose channels (x, z, pitch, hip, knee) plus noisy wheel encoder rate. Both posture and compact jump feedback use estimated state, not true velocities. True state/contact forces serve visualization and evaluation. The added encoder is a simulated sensing assumption, not calibrated hardware. Jump uses time-varying feedback/scheduled observation followed by terminal balance; no nonlinear optimizer runs online.

Horizontal disturbance peaks are 0.5 / 1.5 / 4 N, and pitch moments 0.02 / 0.05 / 0.1 N·m. Push/twist durations are 0.1 / 0.2 / 0.2 s; gust durations 0.2 / 0.25 / 0.2 s. Force acts at the torso origin in world coordinates; moment is about world y. The planar generalized wrench is `[Fx,Fz,Ty,0,0,0]`. Pulse timing and actual wrench are logged.

## Reproducible checks

`node tests/test_wheelbot_actions.mjs` runs the fixed 24-case, 20-second posture/disturbance protocol and separately retains six stronger-pulse outcomes. Final-two-second limits are 0.03 m horizontal error, 0.01 m height error, 0.04 rad pitch error, joint excursion ≤0.02 rad, penetration ≤0.005 m, actual motor ceilings, body rates ≤0.3 and wheel rate ≤1 rad/s. `python tests/test_wheelbot_pose.py` independently checks native trims, linearizations and actual WASM transitions.

Compact jump: `python tests/test_wheelbot_action_jump_reference.py`, `node tests/test_wheelbot_action_jump_wasm.mjs`, and `node tests/test_wheelbot_actions_jump_manager.mjs`. These verify noisy-measurement cases, actual flight, physical limits, final settling and state-preserving admission. They do not certify arbitrary poses or strong disturbances during flight.

`tests/test_wheelbot_actions_browser.py` checks actual page controls, changed targets, wall-clock play, same-robot jump, disturbance units, falling physics, reset/recovery distinction and optional-profile failure isolation. Existing CartPole and reference-wheelbot checks remain. Real-time factor is simulated time divided by measured wall time; FPS/timing observations are not hardware or worst-case guarantees. Physics/control are separate from rendering; full history is not recomputed every frame and analysis updates only while open.

## Earlier experiments and provenance

The collapsed analysis section retains **Reference · standing / jump**, with the earlier model's own MPC/jump/pulse tests. **Try contact lift** uses its distinct full-contact reference model for 0.5 s small lift/hold with exact-state feedback, not full get-up. Old gains are never treated as valid for the compact robot.

The advanced geometry editor belongs to the reference primitive model. Explicit model edits invalidate incompatible profiles. It is not automatic retuning or a universal importer.

Selected leg lengths, wheel dimensions and motor limits derive from the pinned Upkie description. The [source record](../assets/wheelbot/source/provenance.json), [URDF](../assets/wheelbot/source/upkie.urdf) and [Apache-2.0 licence](../assets/wheelbot/source/LICENSE) are retained. The compact torso/lumped drive are declared primitive assumptions. Transmission compliance, battery/thermal limits and real sensor calibration are outside validation. MuJoCo is the reusable physics engine; this is a model-specific lab, not arbitrary-asset compatibility or a registered WebMCP interface.
