# Normal operation of the small-robot learning model

The requested tasks are static balance, 3 cm position regulation, a light recoverable push, and the already verified planned jump. The simulated primitive mass is **1.19915 kg**, excluding motor assemblies and electronics. Ideal torque actuation and source-derived limits (hip/knee/wheel ±16/16/1.7 N·m), contact and sensing are engineering assumptions, not a validated real robot.

The normal push remains **±0.8 N for 0.2 s at the explicit hip origin**, using the existing recovery profile. It is a conservative **model-specific nominal setting**, not a measured touch-force distribution, physical maximum, or hardware capacity claim. The owner supplied no hardware loads. **40 N was introduced by the assistant, not requested by the owner.** It is archived research only: no normal UI option, mandatory recovery goal, release blocker, or fresh optimization target.

## Declared assumptions and evaluated performance

Before any new regression evidence, we declare these low-speed test-design sanity budgets: horizontal acceleration scale `|F|/m <= 0.1g` and free-impulse speed scale `|F|dt/m <= 0.15 m/s`. They are engineering assumptions, not standards, physical proofs, or new controller pass limits. At the retained setting, the scales are approximately **0.068g and 0.133 m/s**, with impulse **0.16 N·s**. They are scale estimates, not predicted constrained-contact accelerations or measured velocities.

The value 0.8 N was historically success-selected: the archived amplitude screen passed all rows at 1 N, then applied a declared 20% reduction. We retain that provenance rather than claiming the value came from real-world force data. Its 20 frozen conditions comprise both signs, LQR/KF and MPC/KF, and seeds 901/907/911/919/929. All 20 historically met the grounded recovery task. Replaying them is regression, not a new independent holdout or a safety probability estimate.

The frozen protocol applies the pulse starting at sample 100 for 20 samples in a 300-sample trial, at 10 ms per sample. Existing final-50-sample limits remain 15 mm position and 0.025 rad pitch; normal recovery requires continuous wheel contact. Existing physical envelopes, gains, sensors, mass and torque limits are unchanged. Position regulation and planned jumping retain their existing task contracts. None establishes arbitrary-impact recovery or hardware transfer.

## Normal UI and routine verification

The signed reset-and-pulse buttons explicitly reset to equilibrium and run only the normal setting. Public `prepareForce` rejects other levels before changing state. Reset restores standing settings. Jump remains a separate planned motor-only trajectory with no external boost or mid-flight reset. When wheel contact is zero, the unchanged kinematic diagnostic is labelled airborne surface velocity, not ground slip.

Routine CI replays only the frozen 20-condition normal force regression:

```sh
node tests/test_wheelbot_operating_scope.mjs
node scripts/validate_wheelbot_force.mjs --operating-only
python tests/test_wheelbot_force_reference.py --operating-only
```

The runner binds the historical receipt, existing protocol, profile and XML identities; it writes `evidence/wheelbot_force_operating_validation.json` and new raw results under `test-results/`. The independent native checker reconstructs every step, force impulse, KF mean, contact and task outcome. Browser test code runs in normal authorized CI only; local HTTP/socket execution remains prohibited. Stale receipt checks disable only the pulse feature.

## Archived research provenance

[Original 160-evaluation receipt](../evidence/wheelbot_force_envelope.json) and [original native receipt](../evidence/wheelbot_force_reference.json) remain unchanged. They retain the screen and failed 40 N cases without reinterpretation. The no-flag runner and native checker remain explicit manual research commands; they regenerate historical output paths and are not routine CI requirements. Some older software regression commands still execute historical 40 N cases to verify recorded failure behavior. That does not make recovery from 40 N an owner requirement or a release gate.
