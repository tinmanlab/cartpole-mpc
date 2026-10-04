# Experiment design and acceptance

A controller can pass many random seeds and still be uninformative if the experiment does not isolate the hypothesis.

## 1. Write the hypothesis first

Examples:

- "The fixed linear model fails because the state leaves the upright local region."
- "The estimator fails because leg/FK measurements are unreliable during slip."
- "The controller fails because applied torque lags the requested torque."
- "The solver fails because the 99th-percentile runtime exceeds the control period."

Then design the smallest A/B experiment that changes only that assumption.

## 2. Separate identification data from evaluation data

System identification needs excitation.

A trajectory that stays almost perfectly upright may be excellent for control but useless for identifying:
- pole length,
- mass,
- friction,
- actuator lag.

Use persistently exciting commands within a safe envelope, then evaluate the identified model on held-out trajectories.

## 3. Check identifiability, not only optimizer convergence

An optimizer returning a number does not mean the parameter was identifiable.

Warning signs:
- high correlation between parameter sensitivities,
- large parameter changes with tiny residual changes,
- different parameter combinations fit equally well,
- held-out model error does not improve.

For complex robots, physical parameterization and excitation design are part of SysID.

## 4. Paired comparisons

Use the same:
- initial state,
- random seed,
- disturbance,
- reference,
- plant realization

when comparing two algorithms.

Otherwise variance can be mistaken for an algorithmic effect.

## 5. Metrics are layered

Controller:
- task/tracking,
- failure rate,
- recovery,
- state/constraint margins,
- control effort and smoothness,
- command vs applied effort,
- solver deadline.

Estimator:
- RMSE / drift,
- innovation,
- NIS / NEES,
- bias,
- covariance calibration,
- update rejection/reliability,
- runtime.

Model:
- one-step prediction,
- rollout prediction,
- held-out trajectories,
- physical plausibility of parameters.

## 6. Mean is not enough

Report at least:
- mean/median,
- worst or high quantile,
- failure count,
- condition/context.

For safety-sensitive deployment, rare-tail behavior often matters more than average score.

## 7. Controller and estimator authority must remain visible

Keep separate:
- truth used only for scoring,
- measurements available to the estimator,
- estimator output available to the controller,
- controller command,
- applied actuator command,
- plant state.

This prevents simulator-only information from leaking into a supposedly realistic loop.

## 8. Acceptance versus diagnosis

Diagnosis experiment:
- narrow,
- one factor,
- may use oracle arms.

Acceptance experiment:
- realistic combined stack,
- held-out,
- no oracle leakage,
- timing enabled,
- constraints enabled,
- wider uncertainty than tuning if appropriate.

Do not use one experiment for both purposes.

## 9. Transfer to humanoids

The same experiment contract scales even though the equations do not.

Replace CartPole-specific quantities with:
- floating-base state,
- joint state,
- contact wrenches,
- torque/current command,
- real IMU/encoder signal contract,
- contact reliability,
- rigid-body/contact solver,
- real scheduler and transport.

The test design stays the same:
one assumption per diagnostic arm, then combined held-out admission.

## Units, selection effects and interpretation

The truth-scoring rule applies to non-oracle admission arms. Explicit oracle diagnostic arms feed truth to the controller and must be labelled. The current simulated R_e calibration also uses injected-noise knowledge. No hardware-ready information contract follows from a realistic-looking plant.

Use positionTrackingRmse for position-reference tracking and estimationRmseByState for [m,m/s,rad,rad/s] errors. Historical rmseState is a mixed-unit estimator aggregate, not physical position error or a controller score. DONE is envelope completion, not target achievement or certified safety. Per-quantity plot scales are independently labelled; visually equal amplitudes across bands do not imply equal physical magnitudes.

For early termination, preserve the failure outcome and compare common prefixes or both-completed full durations with exclusions. A set reused for design/admission is no longer untouched final-test data. Chi-square assumptions, gauge rank and serial dependence matter for NIS/NEES. A finite deterministic grid is not an independent random sample proving a rare-failure bound.
