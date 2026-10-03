# Observers and learned estimator lineage

The sequence on this page is a map of modeling ideas, not a claim that each item is a universal upgrade over the previous one.

## Discrete-time contract used by the lab

The live loop is timestamp-aligned as:

    x_hat_k
       ↓ controller
      u_k
       ↓ plant
    x_(k+1)
       ↓ sensor
    y_(k+1)
       ↓ observer predict/update using u_k
    x_hat_(k+1)

Truth, measurement, and estimate stored in one trace row therefore refer to the same post-step time.

## Raw

The raw baseline measures position and angle and differentiates them:

    v_hat ~= (p_k - p_(k-1)) / dt
    omega_hat ~= wrap(theta_k - theta_(k-1)) / dt

This exposes noise amplification immediately. No calibrated covariance P is claimed for this finite-difference output.

## Kalman Filter

The KF uses the upright discrete linearization of the same shared plant transition.

    x_hat_minus = A x_hat_plus + B u
    P_minus = A P_plus A^T + Q

    residual = y - H x_hat_minus
    K = P_minus H^T (H P_minus H^T + R)^-1
    x_hat_plus = x_hat_minus + K residual

Definitions:
- P: covariance of the current state-estimation error under the filter model,
- Q: configured process/model uncertainty,
- R: measurement-noise covariance.

For the Gaussian-noise scenarios in this lab, R is initialized from the actual injected measurement standard deviations for that scenario. Q remains a deliberately tuned educational process covariance; it is not claimed to be identified from hardware data.

The covariance correction uses the Joseph form:

    P_plus = (I-KH) P_minus (I-KH)^T + K R K^T

which is more numerically robust than the shorthand (I-KH)P.

## EKF

The EKF keeps the Kalman update structure but propagates the mean through nonlinear dynamics:

    x_hat_minus = f(x_hat_plus, u)
    F = df/dx at the current estimate
    P_minus = F P_plus F^T + Q

The runtime uses a central-difference Jacobian of the same nonlinear 20 ms plant transition.

## Invariant filtering bridge

InEKF is not simply "EKF but newer."

When the state and dynamics have suitable Lie-group / group-affine structure, an invariant error can yield error dynamics with useful autonomy/log-linear structure and improved consistency/convergence properties.

The CartPole runtime can honestly demonstrate only the SO(2) angle-error issue:

    angle residual = wrap(theta_measurement - theta_estimate)

That mode is therefore named an SO(2) error bridge, not a CartPole InEKF implementation.

The Hartley contact-aided InEKF instead operates on a floating-base Lie-group state containing orientation, velocity, position, and active contact positions, fusing IMU propagation with leg forward-kinematics observations. Global translation and yaw remain unobservable without additional absolute information.

## Lin et al. — learned contact events

For the Mini Cheetah implementation in the paper:
- each synchronized frame contains 54 values: joint position/velocity, IMU acceleration/angular velocity, and foot position/velocity from kinematics,
- the network consumes a 150-frame history,
- two Conv1D blocks plus three fully connected layers classify the 16 possible four-leg contact configurations,
- the class is decoded into per-leg binary contact events,
- contact positions are added to or removed from the contact-aided InEKF state accordingly.

So "learned binary contact" describes the downstream event semantics, not a network with one independent binary output per foot in that experiment.

## Youm / Neural Measurement Network

The NMN is a GRU-MLP using:

    [acceleration, angular velocity, joint position,
     joint velocity, previous desired joint position]

It predicts:
- per-foot contact probabilities,
- body-frame linear velocity.

The contact probability gates the leg-kinematics measurement. The learned body velocity is formulated as a right-invariant IEKF measurement.

The reported network uses GRU hidden size 128 and MLP layers [256, 128]. In hardware inference the outputs are low-pass filtered; contact is thresholded at 0.5, and the learned velocity measurement is conditionally suppressed at very low speed to limit bias-induced drift.

## InNKF

InNKF keeps the model-based InEKF and adds an output compensation stage.

The paper's neural compensator:
- consumes a 50-step / 0.1 s history at 500 Hz,
- uses a TCN with hidden sizes [128, 128, 128, 256, 256],
- predicts 9 tangent-space coefficients,
- maps them through the se_2(3) generators and exponential map into an SE_2(3) error element,
- corrects the updated InEKF base state.

Conceptually:

    InEKF posterior X_bar_plus
            ↓
    neural error E_hat
            ↓
    X_bar_plusplus = E_hat^-1 X_bar_plus

Critical authority boundary: the compensated X_bar_plusplus is the final output and is not fed back into the base InEKF state.

The CartPole runtime follows that authority boundary. Its small residual MLP corrects only the reported output; the base EKF continues independently.

The base EKF covariance P is not claimed to be the covariance of the neural-corrected output.

## CoCo-InEKF

CoCo-InEKF does not merely inflate a measurement R.

Its differentiable InEKF permanently maintains predefined contact-candidate positions in the state. A neural contact module predicts a body-frame contact-candidate velocity/process covariance for each point.

For each 3-D candidate, the network predicts six lower-triangular entries:

    L_i
    Sigma_Ci = L_i L_i^T

which guarantees a symmetric positive-semidefinite covariance.

That covariance enters the contact-position process model, continuously expressing confidence from firm/stationary through directional slip to uncertain/no-contact behavior.

Because CartPole has no persistent foot/contact-candidate states, this repo provides a theory page for CoCo and does not pretend that the Adaptive-R mode is a CoCo implementation.

## FOCUS

FOCUS asks:

    "How reliable is this foot's FK-derived body-velocity observation right now?"

rather than only:

    "Is the foot in contact?"

The paper predicts continuous per-foot reliability weights from a causal Transformer using a 50-frame history of sensor-only proprioception:

    [IMU acceleration, IMU angular velocity,
     lower-limb joint position, joint velocity]

The reliability affects both the velocity observation and its covariance. In simplified notation:

    R_vel,i = R_vel,0 [1 + (1-w_i) S_vel]
    tau_i = clip(w_i / w_sat, 0, 1)
    z_v,i = (1-tau_i) v_IMU,pred + tau_i v_FK,i

The paper also modulates foot-position/height observation covariance and foot-state process noise.

The CartPole Adaptive-R mode is intentionally narrower:
- it uses an innovation-based outlier score, not a learned Transformer,
- it changes the position/angle observation R,
- it does not have feet, FK velocity blending, or foot-state process noise.

Because R is chosen from the same innovation being processed, this mode is a heuristic robustification, not the classical fixed-noise Kalman filter. Its P should be read as the covariance propagated under that adaptive heuristic, not as a hardware-calibrated uncertainty guarantee.

It is therefore a FOCUS-style observation-reliability bridge, not a reproduction of FOCUS.

## What to compare in this lab

Useful comparisons:
- Raw vs KF under sensor noise: differentiation noise versus model-based fusion.
- KF vs EKF with **Estimator nonlinear bench** and LQR fixed: the pole starts at 0.5 rad with very low sensor noise and no external push. In the fixed five-seed check, KF state RMSE is about 0.0080 and EKF about 0.00323. This bench is not a controller ranking.
- EKF vs SO(2) bridge with the angle-wrap micro-bench: +179 deg and -179 deg are 2 deg apart on SO(2), while an unwrapped Euclidean residual is about -358 deg.
- KF/EKF vs Adaptive-R under a measurement glitch: outlier down-weighting.
- Base EKF vs residual output: offline residual fit versus closed-loop control benefit.

Do not infer paper superiority from these CartPole results.
