# Constraint diagnostic receipt

Receipt SHA256: `c94ffbdb072b805a4bbe2ae0f75c983ecb3b03678693ada6581c20eafdbaa04f`

Protocol SHA256: `831d95588e02c51ee66bc67a9f1125f57cbe5a6fb842f4f3457da6865285d8e9`

Physical results: 4/20 task passes; 4 completed; 16 MPC rail-active samples.

| Case | Pair | Outcome | Applied steps | Task passed |
|---|---|---|---:|---|
| interior | lqr-kf | completed | 600 | True |
| interior | lqr-ekf | completed | 600 | True |
| interior | mpc-kf | completed | 600 | True |
| interior | mpc-ekf | completed | 600 | True |
| right-near-rail | lqr-kf | envelope-failure | 5 | False |
| right-near-rail | lqr-ekf | envelope-failure | 5 | False |
| right-near-rail | mpc-kf | solver-rejected | 3 | False |
| right-near-rail | mpc-ekf | solver-rejected | 3 | False |
| left-near-rail | lqr-kf | envelope-failure | 5 | False |
| left-near-rail | lqr-ekf | envelope-failure | 5 | False |
| left-near-rail | mpc-kf | solver-rejected | 2 | False |
| left-near-rail | mpc-ekf | solver-rejected | 2 | False |
| right-model-error | lqr-kf | envelope-failure | 6 | False |
| right-model-error | lqr-ekf | envelope-failure | 6 | False |
| right-model-error | mpc-kf | solver-rejected | 0 | False |
| right-model-error | mpc-ekf | solver-rejected | 0 | False |
| left-model-error | lqr-kf | envelope-failure | 5 | False |
| left-model-error | lqr-ekf | envelope-failure | 5 | False |
| left-model-error | mpc-kf | solver-rejected | 4 | False |
| left-model-error | mpc-ekf | solver-rejected | 4 | False |

Offline initialization ablation; each row compares the same nominal QP at one physical time. Truth and its plans are evaluator-only. Step-zero initial/rejected duplicates are combined.

| Case | Pair | Step | Continuous (native) | Cold reset (native) | Truth (native) |
|---|---|---:|---|---|---|
| interior | mpc-kf | 0 | accepted | accepted | accepted |
| interior | mpc-ekf | 0 | accepted | accepted | accepted |
| right-near-rail | mpc-kf | 0 | accepted | accepted | accepted |
| right-near-rail | mpc-kf | 3 | solver-declared infeasible | initial-state-outside | solver-declared infeasible |
| right-near-rail | mpc-ekf | 0 | accepted | accepted | accepted |
| right-near-rail | mpc-ekf | 3 | solver-declared infeasible | initial-state-outside | solver-declared infeasible |
| left-near-rail | mpc-kf | 0 | accepted | accepted | accepted |
| left-near-rail | mpc-kf | 2 | solver-declared infeasible | initial-state-outside | accepted |
| left-near-rail | mpc-ekf | 0 | accepted | accepted | accepted |
| left-near-rail | mpc-ekf | 2 | solver-declared infeasible | initial-state-outside | accepted |
| right-model-error | mpc-kf | 0 | initial-state-outside | initial-state-outside | accepted |
| right-model-error | mpc-ekf | 0 | initial-state-outside | initial-state-outside | accepted |
| left-model-error | mpc-kf | 0 | accepted | accepted | accepted |
| left-model-error | mpc-kf | 4 | solver-declared infeasible | initial-state-outside | solver-declared infeasible |
| left-model-error | mpc-ekf | 0 | accepted | accepted | accepted |
| left-model-error | mpc-ekf | 4 | solver-declared infeasible | initial-state-outside | solver-declared infeasible |

Solver-declared infeasibility is a solver status, not a mathematical certificate or proof of plant impossibility. Numerical nonacceptance is separate. Jointly accepted JS/native plans are checked for first action and original cost. No measured successful warm-up is claimed.

Position intervals in the receipt are mean +/- 2 sqrt(Ppp) under assumed covariance, with same-time replay covariance identity and evaluator-only error. They are not calibrated risk, robust margins, safety permission or simultaneous coverage. Zero-transition residuals are null with zero samples. No online use or default changes.

Source SHA256 bindings:

- `assets/cartpole.xml`: `0cd42edcef6ecccb9128af20644a17cfbe2ea484208675458c161d01cc40e46e`
- `scripts/constraint_diagnostic_report.py`: `24d1065310210702abc908e8085c01f1ac10c8ff7db19b03d3cc9bcf2a895f20`
- `scripts/run_constraint_diagnostic.mjs`: `f4703fa5522bb39c19fb2d953c847dd39ec8808519b3919b5d6b02f021061690`
- `src/calibration_lab.js`: `6292666d655dc6a995de433e31e5203b0c291a944223b8ae517274a0ff5a6221`
- `src/design_study.js`: `2a8a7d38a03e98ff017e4bfeef430b4d636bf01cdd5c105891a6effdd5bf02bb`
- `src/engine.js`: `516cb2ad4cf66f1a823c26c515cd237c45c5f4858b992aa6b3ff80bbbdfc10b3`
- `src/mujoco_backend.mjs`: `f7aea47b45e6ec0a0372e94e166e813d0a5429f8ea6193672539fedf001f0444`
- `src/plant.js`: `671d502bbac9e03c83e1372246c1abafc0b7851a03ee2408c548bac39faec34d`
- `src/qp.js`: `521c56220a2816e41114f008de22a89cec5175f34e6e6c57ed80c07e533f472a`
- `tests/fixtures/constraint_diagnostic.json`: `831d95588e02c51ee66bc67a9f1125f57cbe5a6fb842f4f3457da6865285d8e9`
- `tests/fixtures/design_study.json`: `1cc2de2d8d72fa151a5b5e5aa9ef2659c5f9783d7aff6ea2d56ea791b5e79445`
- `tests/osqp_condensed_reference.py`: `3182ed9b975bdfd479bb03a92546e8ed66136edaa2eca2c8d59bd6d8c0047d8d`
- `tests/test_constraint_diagnostic.py`: `6304aa1931d4ab48d7ac593491b0b30732cfcb3a1fafff90ee91c93a111a07c9`
