# Observers: model, information, uncertainty and comparison

The lesson order is not a universal performance ranking. Controller costs Q_c/R_c/P_f and estimator Q_e/R_e/P have distinct meanings. The UI names both the lesson's scope and the actual running configuration.

## Time, information and missing data

The simulated loop is x_hat_k -> u_cmd_k -> plant x_(k+1) -> y_(k+1) -> x_hat_(k+1). This post-step row alignment is not an asynchronous acquisition/arrival timestamp implementation. Delayed and out-of-sequence fusion remain separate tasks.

The estimator receives requested command, not privileged realized actuator force. Lag/gain/delay therefore create unknown-input error. In normal observer modes the controller receives an estimate. **Truth/oracle is an exception: it feeds simulator truth to the controller.** Its zero error/P is not a calibrated sensor or hardware confidence interval.

Scenario R_e is initialized from injected simulation variance; offline ID/training can also use truth. This is disclosed simulation knowledge, not sensor-only calibration. Known dropout triggers prediction-only updates, not repeated assimilation of a held packet. A delivered frozen sensor is a different fault.

Raw differences use the elapsed interval between valid measurements. For independent equal-variance errors, velocity-noise variance is 2*sigma²/dt²; temporal correlations alter this expression. No calibrated output P is claimed.

## KF and EKF

The standard discrete linear, independent-noise equations are:

    x_minus = A x_plus + B u
    P_minus = A P_plus Aᵀ + Q_e
    innovation = y - H x_minus
    S = H P_minus Hᵀ + R_e
    K = P_minus Hᵀ S⁻¹
    x_plus = x_minus + K innovation
    P_plus = (I-KH) P_minus (I-KH)ᵀ + K R_e Kᵀ

Correct Gaussian prior/noise and linear modeling give a conditional-mean/covariance interpretation. Under appropriate second-moment assumptions without Gaussianity, a linear minimum-variance claim is narrower. Correlated noise needs modified equations. Joseph form improves numerical handling, not a wrong physical model. Cross covariance couples position/angle corrections into velocities; it does not make an unobservable state observable.

EKF propagates its mean through the nominal nonlinear MuJoCo transition and covariance through a numerical Jacobian. The actual measurement is linear selection H=[p,theta]; a general nonlinear measurement h needs its own Jacobian/error coordinates. The base EKF keeps an unwrapped measurement residual for comparison. The SO(2) mode wraps that residual and posterior angle. Ordinary EKFs can wrap angles: this alone does not implement a Lie-group invariant filter.

## UKF

Deterministic sigma points propagate through the nonlinear transition and are recombined, with circular angular means. They are not random Monte Carlo samples. A nonlinear mapping generally does not preserve Gaussianity; one local mean/covariance is an approximation, not an exact posterior or a universal upgrade over EKF.

For THIS position/angle sensor in its local chart, the measurement update projects the complete predicted P including additive Q_e:

    P_xz = P_minus Hᵀ
    S = H P_minus Hᵀ + R_e

No measurement sigma-point transform is executed here. Using only pre-Q propagated points would omit process noise in S/P_xz. General nonlinear h requires a consistently reconstructed or augmented sigma-point formulation. Wide circular or multimodal beliefs remain outside a single local-covariance description.

## MHE

MHE is a separate windowed estimation design, not a mandatory successor to EKF/InEKF. It can be linear or nonlinear; hard constraints are optional formulation choices. An arrival cost summarizes earlier information.

The local deterministic shooting example uses eight transitions, **0.16 s**, and optimizes only four components of the oldest state. Its correlated EKF arrival prior is approximate; the prior's measurement is not counted again. There is no optimized process-noise sequence, hard state constraint or calibrated MHE output covariance. Base EKF P is not that missing output covariance. A window-fit residual is post-fit, not the prefit innovation used by standard NIS.

The cited EKF+MHE paper reports an orientation/velocity split using OSQP at 200 Hz with a 0.1 s window. These are that experiment's choices, not this browser's speed or a universal MHE configuration.

## Invariant and learned branches

| Reference | Interface studied | Local scope |
|---|---|---|
| Hartley contact-aided InEKF | invariant error and IMU/contact kinematics | wrapped-angle EKF analogue only |
| Lin | learned contact events | paper explanation, no classifier reproduction |
| Youm NMN | learned contact/velocity measurements | paper explanation, no robot/network reproduction |
| InNKF | temporal compensation of invariant posterior | output-only residual MLP around SO(2)-aware EKF |
| CoCo-InEKF | contact-candidate process covariance | paper explanation; no contact-state implementation |
| FOCUS | FK observation reliability and velocity fusion | narrower innovation-based R_e heuristic |

Invariant-error benefits require the relevant group-affine/symmetry assumptions; bias/contact errors complicate them. Without absolute information, the cited contact-inertial model retains global translation/yaw gauge. These are not properties of every possible invariant filter.

Residual output is not fed back into base EKF, but the controller uses it, so feedback through the plant still exists. Base P is not corrected-output P. Offline residual fit does not prove closed-loop benefit.

CoCo's L Lᵀ ensures positive semidefiniteness, not strict positivity or calibration automatically. No direct contact labels does not mean no state-ground-truth supervision: its reported state-error training uses simulated state information.

FOCUS was verified at official abstract level in this review. Detailed widths/thresholds/scale equations were not independently rechecked and are not prescribed. Adaptive-R chooses R_e from the same innovation; classical fixed-noise KF optimality and exact chi-square coverage do not transfer unchanged. Learned pseudo-measurements sharing sensors with the prior can also be correlated with it.

## Statistics and physical units

NIS uses prefit innovation/S; NEES uses same-time state error/P. Under correctly modeled Gaussian assumptions, the quadratic forms have chi-square laws in their effective nonsingular coordinates. For singular covariance/gauge, state the tested rank/coordinates and treatment rather than using an ordinary inverse silently. Serial samples are not automatically independent trials.

Check bias, whiteness, coverage and repeated trials; an average near the nominal dimension is insufficient. For nonlinear/adaptive filters these are diagnostics unless stronger assumptions are justified. Missing covariance/statistics remain unavailable, not perfect scores.

`estimationRmseByState` reports [p,v,theta,omega] in [m,m/s,rad,rad/s]. `positionTrackingRmse` compares true position with the reference. Legacy `rmseState` sums heterogeneous units and is retained only for historical compatibility, not a controller ranking. Plots label quantity-specific scales; the UI no longer forms a mixed-unit innovation norm.

Continuous noise spectral density is not discrete Q_e. For continuous linear dynamics, Q_d is the integral of exp(A*t) G Q_cont Gᵀ exp(Aᵀ*t) over the sample interval. Changing dt generally changes Q_d. Persistent bias and model mismatch may require augmented states/identification, not arbitrary covariance inflation.

## Primary references

- KF: https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html
- Hartley: https://arxiv.org/abs/1904.09251
- Lin: https://proceedings.mlr.press/v164/lin22b.html
- Youm: https://arxiv.org/abs/2402.00366
- InNKF: https://arxiv.org/html/2503.00344v1
- EKF+MHE: https://arxiv.org/html/2405.20567v1
- CoCo: https://arxiv.org/html/2605.15122v1
- FOCUS (abstract-level verification): https://arxiv.org/abs/2609.02222

Paper-specific network sizes and old benchmark numbers are not current general facts. Dated receipts keep their recorded source/configuration; use fresh tests for current implementation outcomes.
