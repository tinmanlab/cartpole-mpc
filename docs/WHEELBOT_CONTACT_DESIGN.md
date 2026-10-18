# Editable wheelbot mechanics and actual full-body contact

This extension is an explicit physical-design mode in the same `wheelbot.html` view. It does not overwrite the older wheel-contact-only benchmark, its identified hashes, gains, pulse protocol or planned jump. The purpose is to connect model geometry/inertia/actuator assumptions to contact behavior and subsequent controller redesign.

## Change and test a physical design

Open **Robot design · base, links, actuators and full collision physics**. Common fields expose torso mass/width/height, individual link length/mass, wheel radius, each motor torque ceiling, joint viscous damping, reflected rotor inertia and sliding friction. The advanced JSON includes remaining radii/widths, joint limits, contact compliance and obstacle dimensions/location. The fields' editor ranges are finite validity bounds, not verified hardware limits.

Apply validates parameters and recompiles the actual MuJoCo WASM model. The shape displayed, mass/inertia, link/joint positions, torque limits and contact geometry come from those same compiled bytes. The three torque motors remain the only actuators; root x/z/pitch remain unactuated. The model still assumes ideal torque actuation. A changed torque ceiling is not a complete motor torque-speed, latency, thermal or compliance model.

The configuration file is `assets/wheelbot/contact_design.json`. One shared JavaScript builder, `src/wheelbot_configuration.mjs`, transforms the pinned original XML for both the browser and native-test CLI. It recomputes solid-box and solid-cylinder inertias with mass/dimension edits. The browser can import/export JSON and export the actual standalone MJCF. There is no separate hidden browser plant or hand-coded multibody dynamics.

All torso, upper-link, lower-link and wheel geoms have collision bits enabled. They interact with the plane and the configurable fixed box obstacle. MuJoCo's normal parent-child collision filter excludes the directly connected hinge neighbors; nonadjacent self-contact remains enabled. This adjacency treatment is a modeling assumption, not proof that real joint housings never interfere. Current primitives omit motor housings/electronics and are not a hardware digital twin.

The physical trial can fall and remain on the ground. Unlike the standing controller's local task, it does not stop merely because pitch exceeds 0.6 rad or root height drops below 0.12 m. Actual contact-pair names, normal forces and penetration distances are shown. Soft-contact penetration is measured, not silently asserted to be zero. Observed drop transients can penetrate about 10–16 mm in this coarse compliant primitive model; contacts are not a hard nonpenetration certificate. The default obstacle is a declared small geometric test, not an inferred real environment.

**Reset & fall left/right** changes the initial condition and then lets gravity act. It does not inject a force, freeze the robot at impact, or teleport it upright. **Reset & torque test** applies explicitly entered motor torques with the compiled ceilings. **Reset & local LQR/KF test** uses an exact-model local profile and resets to its standing trim; that is not a stand-up-from-the-floor demonstration.

## Physics changes invalidate old controller assumptions

After applying a model, the old benchmark's control, pulse and jump selectors are disabled and their public API requests are rejected. A pre-generated local profile is available only for the exact default full-contact XML. A different configuration requires a new trim, linearization and gain calculation; importing a stale/mismatched profile is rejected.

To regenerate a configuration and a local profile using the existing native environment:

```
node scripts/build_wheelbot_contact.mjs path/to/config.json assets/wheelbot/custom.xml
python scripts/design_wheelbot_contact_profile.py --model assets/wheelbot/custom.xml --output assets/wheelbot/custom_profile.json
```

The native command reuses the repository's existing MuJoCo/SciPy LQR/KF designer instead of introducing another controller. A defect exposed by shortening the links was fixed: the trim solver previously started at the old hard-coded 0.54 m height, leaving the new shorter mechanism above the ground, where height derivatives could not recover the contact equilibrium. Variant initialization is now computed from the actual compiled link lengths and wheel radius. The original designer's default path is unchanged. Torque limits are checked against the solved feedforward torque.

The profile is local. Validating its Riccati residual and local poles is not a certificate for falls, obstacles or jumping. Restore verified benchmark explicitly returns to the original model and previously tested profiles.

## Full-contact and independent checks

The model tests compare default and edited mass/link/torque configurations. They execute left/right drops, obstacle contact and a one-step torque-limit diagnostic, totaling 2,402 control steps. The native reference reconstructs each step using the same XML and torques, and independently recomputes contact names/distances/world forces, body inertias and local Riccati gains. Native/WASM maximum state-component disagreement was approximately 7.89e-13 and normal-force disagreement 3.40e-12 N in the recorded run. These numbers demonstrate implementation agreement, not empirical hardware fidelity.

A WASM output-buffer defect was caught during verification: passing an ordinary JavaScript Float64Array to the native contact-force function left the displayed forces at zero. The diagnostic now uses the upstream `DoubleBuffer`, and a positive load-bearing-force assertion plus native force comparison prevent a fake zero-force display. The second compiled model in the diagnostic wrapper is used only for forward/geometry/force inspection; the existing backend remains the sole integrator.

## Can it get up from a fall?

The contact model now permits a meaningful test, unlike the old model whose body/link contacts were disabled. The first bounded feasibility study uses a physically settled fallen pose and actual three-motor commands. It compares the source-sized model, a shorter/lower torso, and a compact-link/higher-damping variant. Each uses the same 196-evaluation search budget for joint-target sequences and bounded wheel-rate targets; a local LQR is handed control only after a near-standing state is reached.

This is **privileged full-state offline research**, not an implemented sensor-based get-up controller. None of the three designs produced a candidate satisfying all declared stand-up, settling, joint-limit and penetration criteria in this bounded search. The lower-body candidate's smaller angular error also came with excessive contact penetration; it is not an approved improved design. No mass, range, torque or contact change has been promoted as an optimal hardware redesign.

Failure of this particular controller family and search budget is not a proof of physical impossibility. It does show why a standing linear controller and a smooth joint interpolation cannot simply be called a get-up controller. The motion needs a feasible sequence of supporting contacts, collision-free joint configurations and controlled transfer into the standing region. The next research decision must be supported by such a trajectory or a meaningful feasibility bound, not by endlessly increasing torque or by deleting body collisions.

`evidence/wheelbot_selfright_study.json` preserves the exact parameter variants, search bounds, failed outcomes, model hashes and scope. `python scripts/study_wheelbot_selfright.py` reproduces the bounded experiment; it is not automatically repeated in every CI run. This preserves the user's intended normal-scale robotics task without inventing a high-force requirement.

## Method references and scope

The implementation uses official MuJoCo 3.7.0 contact filtering, soft constraints, actuator limits and contact-force interfaces:
- https://mujoco.readthedocs.io/en/3.7.0/XMLreference.html#contact
- https://mujoco.readthedocs.io/en/3.7.0/computation/index.html#collision-detection

The distinction between contact modes and fixed linearization follows the attached control-estimation commissioning material. No new RL framework, generalized plant control plane or physics engine is introduced. Configurable mechanics, full-contact simulation, local redesign, stand-up feasibility, observer-based execution and final hardware claims are separate acceptance levels.

## Reuse decision: published get-up methods before further ad hoc search

The current joint-sequence search is not a test of a contact-aware MPC stack. Further structural redesign should not be justified solely by that search's failures. Adopt an existing full-dynamics planning implementation first; write only a wheelbot task definition, model/state mapping and independent acceptance adapter.

**Primary model-based donor: MuJoCo MPC (MJPC)**, official `google-deepmind/mujoco_mpc` at `ff572a21e7c2bf9fda62e1862a758da7e9a8719b`, Apache-2.0. Howell et al., *Predictive Sampling: Real-time Behaviour Synthesis with MuJoCo*, arXiv:2212.00541, Section4.2 explicitly demonstrates a humanoid standing from the floor and an A1 rolling from its back to standing. Reuse the actual upstream sampling/iLQG planners, not another handwritten optimization algorithm. Its humanoid task residual is a source pattern, not a valid task file for our three-input planar model. The authors describe predictive sampling as a deliberately simple baseline, not a novel universally best optimizer.

A bounded first adaptation uses our unchanged full-contact MJCF, actual three torque limits, consistent state ordering and a model-specific get-up objective. The nonlinear MuJoCo rollout supplies contact dynamics; a separate handcrafted contact schedule is not mandatory for every sampling-MPC approach. Contact-stage hints can help avoid local minima, but they must be documented as a task/design choice. State/contact safety and success criteria remain separately evaluated; successful cost minimization alone is not a stand-up certificate. Exact-state feasibility and measured-state execution are separate tests. Native upstream control must be rebuilt/verified for the canonical MuJoCo version before comparing with the WASM plant; an existing binary with another physics version is only an upstream diagnostic.

**Mechanism and recovery-stage reference: Ascento.** Klemm et al., *Ascento: A Two-Wheeled Jumping Robot*, ICRA2019, arXiv:2005.11435, SectionsIV-C andV-B3. Its demonstrated recovery proceeds through leg retraction, controlled extension/retraction that changes support and tips the body toward sitting, then wheel torque and LQR stabilization. Three of four resting configurations are addressed, not arbitrary falls. Its optimized bilateral linkage and mechanical parameters differ from our single-side serial hip/knee design. Reuse support-transition/geometry reasoning and transition acceptance, not its gains or an asserted drop-in controller. An author-maintained runnable controller release was not verified in this review.

**Direct recent wheeled-biped research and code: FTSR / GetUp Gym.** Hou et al., *Robust Fall Recovery for Armless Bipedal-Wheeled Robots via Force-Guided Learning*, IEEE RA-L2026, DOI10.1109/LRA.2026.3701481, arXiv:2606.14270. Author-linked code is `2350575870/getup_gym` at `238d2962a0fee8e3da4da914cadcea2be00ce402`. Actual environment/config/teacher-student training files are present, rather than only a project landing page. It targets an8-action bilateral wheel-legged robot and a humanoid, uses Isaac Gym/PyTorch/CUDA and constrained teacher-student RL, not MPC. The proprioceptive actor path and privileged teacher path must stay distinct. Training-only upward assistance is reduced and removed in the paper's final policy; this never authorizes hidden assistance in our evaluation. Use staged upright/posture/locomotion objectives as a research comparison and possible trajectory prior; do not silently switch this model-based project into an RL training campaign. The inspected tree contains no trained-weight files or dedicated license file, although README declares BSD-3-Clause; redistribution/dependency notices need review before copying source. No ready-to-run policy for our three-motor model is established.

**GPU alternative only if measured planning throughput requires it: Hydrax.** `vincekurtz/hydrax` has an actual `examples/humanoid_standup.py`, currently using MPPI-CMA with a MuJoCo MJX model, as well as sampling/CEM/DIAL options and model randomization. Its JAX/MJX/GPU environment and example's intentionally different planning/simulation contact settings are separate compatibility work; do not treat identical XML as proof of identical numerical contact behavior or install a new GPU stack before a demonstrated need.

**State-estimation boundary.** Zhang et al., *Whole-Body Model-Predictive Control of Legged Robots with MuJoCo*, arXiv:2503.04613, provides further whole-body/hardware evidence. The author deployment repository `johnzhang3/mujoco_mpc_deploy` explicitly says its estimator is under development and uses OptiTrack/ROS. It is not a supplied solution for our existing five-channel noisy observer.

The next admissible implementation is one same-version upstream task adapter and one fixed, physically settled fallen initial condition. First establish an actual get-up trajectory with no hidden external force, pose reset, disabled collisions or increased motor limits; then feed the existing measurement history through the appropriate estimator and re-evaluate the same task. Only a specific contact/reachability/torque barrier, not a local optimizer failing, can justify a claim that the mechanism itself must change.

Primary sources:
- https://arxiv.org/html/2212.00541
- https://github.com/google-deepmind/mujoco_mpc/tree/ff572a21e7c2bf9fda62e1862a758da7e9a8719b
- https://arxiv.org/html/2005.11435
- https://arxiv.org/html/2606.14270
- https://2350575870.github.io/force-guided.github.io/
- https://github.com/2350575870/getup_gym/tree/238d2962a0fee8e3da4da914cadcea2be00ce402
- https://github.com/vincekurtz/hydrax/blob/main/examples/humanoid_standup.py
- https://github.com/johnzhang3/mujoco_mpc_deploy
