# External force: an assessed operating task, not a universal disturbance bound

The teaching task now distinguishes a normal local-recovery pulse from a deliberate failure-boundary test. This is an explicit change to the external challenge, not a claimed controller improvement on the old 40 N test. The historical `tests/fixtures/wheelbot_cases.json`, its 3 N × 0.1 s and 40 N × 0.2 s cases, physics asset, recovery gains and sensor noise remain unchanged.

## What was adjusted

The new long-pulse task uses a 0.2 s world-horizontal force at the torso/hip origin, beginning at sample 100 of the same 300-sample, 10 ms experiment. Both signs and both existing controller/observer combinations (LQR/KF and MPC/KF) are tested. Position and pitch targets remain 15 mm and 0.025 rad during the final 0.5 s, with the original 1 m / 0.6 rad / 0.12 m physical envelope. The **normal** task additionally requires continuous wheel contact; this is a declared local operating condition, not the definition of every physically possible recovery.

The model weighs approximately 11.76 N (mass 1.19915 kg). A 40 N, 0.2 s pulse is about 3.40 times the model weight and has a commanded impulse of 8 N s. These are scale comparisons, not proofs of dynamic infeasibility. The force acts approximately 0.125 m below torso COM at the reference posture, so its initial pitch moment about that COM is approximately -5 N m. Moving the point to COM would be another task and was not done.

## Screen, lock, then independent noise assessment

`tests/fixtures/wheelbot_force_envelope.json` fixes the amplitude grid [1,2,3,4,6,8,12,20,40] N, both directions, both controllers and three screening seeds before the experiment. A zero-force baseline is included. Each amplitude has 12 declared condition rows; paired controllers/signs are not independent random samples.

| 0.2 s pulse magnitude | Full-duration completion / 12 | Grounded recovery target / 12 |
|---|---:|---:|
| 0 N | 12 | 12 |
| 1 N | 12 | 12 |
| 2 N | 12 | 2 |
| 3 N | 12 | 0 |
| 4 N | 6 | 0 |
| 6 N | 0 | 0 |
| 8 N | 0 | 0 |
| 12 N | 0 | 0 |
| 20 N | 0 | 0 |
| 40 N | 0 | 0 |

The selection rule takes the largest *contiguous all-pass prefix* of the grid, then selects 80 percent of its upper point, rounded downward to 0.1 N. It cannot skip over a failed amplitude. This selects **0.8 N × 0.2 s**, from the 1 N all-pass screening point. The 20 percent amplitude reduction is a declared engineering margin, not a statistical safety guarantee or a globally optimal force.

The force was locked before seeds 901/907/911/919/929 were opened. At 0.8 N, all 20 sign/controller/noise conditions completed and met the grounded recovery task. The separately retained 40 N stress conditions all failed (0/20). There were 160 total evaluations including screening and both assessments. No controller gain, force point, episode duration, sensor noise or pass threshold was retuned after the assessment.

The selected pulse is 0.16 N s and approximately 6.8 percent of the model's weight. It is intentionally conservative for this particular long-pulse, rapid-settling task. It is not a statement that forces above 0.8 N are physically unrecoverable. The original 3 N × 0.1 s recovery result remains valid in its own shorter-pulse task; force magnitude alone cannot define task equivalence.

## Independent verification

`tests/test_wheelbot_force_reference.py` reconstructs the actual pulse timing/sign/impulse, every KF mean, original LQR action, native one-step physics, contact counts, final-window metrics, screen selection and separate admission. It checks 160 executions / 31,668 applied control samples. Recorded maximum native/WASM state component disagreement is approximately 1.23e-12 and KF mean disagreement 1.03e-13. These validate implementation arithmetic, not model-to-hardware accuracy.

The zero-force rows and paired controller/sign comparisons share inputs, so 160 is not a count of 160 independent safety trials. Five new noise realizations on a fixed physical model do not establish a rare-failure probability, an invariant region or physical out-of-distribution performance. All observed failures remain in `evidence/wheelbot_force_envelope.json`.

## Using the shared wheelbot view

`wheelbot.html` offers an assessed local pulse and the separate 40 N stress pulse, plus signed reset-and-pulse buttons. Starting one explicitly resets to the declared equilibrium, uses the existing recovery profile with the selected LQR/KF or MPC/KF controller, and stops at completion or physical failure. The actual force, duration, point, applied impulse and target outcome are displayed. It does not covertly push during a normal standing or jump trial. Reset restores the user's standing settings.

A mismatched or unapproved force receipt disables only the pulse feature. It is checked against the exact loaded recovery-profile and protocol hashes, and its selection/admission is reconstructed. This is accidental data-consistency validation, not a security signature.

Reproduce with the pinned native environment:

```
node scripts/validate_wheelbot_force.mjs
node tests/test_wheelbot_force_contract.mjs
python tests/test_wheelbot_force_reference.py
python tests/test_wheelbot_force_browser.py
```

The browser test belongs in the authorized CI environment, not a socket-denied developer sandbox. Current deployment/merge state belongs to GitHub, not this methodology note. A successful planned jump is a different experiment and does not turn the historical 40 N failure into a pass.
