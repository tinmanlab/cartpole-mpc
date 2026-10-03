# Observers and learned estimator lineage

## Raw

The raw baseline measures position and angle and differentiates them:

    v_hat ~= delta position / delta t
    omega_hat ~= wrapped delta angle / delta t

This exposes noise amplification immediately.

## Kalman Filter

The KF combines model prediction and measurement correction.

    x_hat_minus = A x_hat_plus + B u
    P_minus = A P_plus A^T + Q

    residual = y - H x_hat_minus
    K = P_minus H^T (H P_minus H^T + R)^-1
    x_hat_plus = x_hat_minus + K residual

P is state-estimation uncertainty.
Q is process/model uncertainty.
R is measurement uncertainty.

## EKF

The EKF keeps the Kalman update structure but propagates the mean through nonlinear dynamics.

    x_hat_minus = f(x_hat_plus, u)
    F = df/dx
    P_minus = F P_plus F^T + Q

## Invariant filtering bridge

The CartPole runtime can honestly demonstrate only the SO(2) angle-error issue:

    angle residual = wrap(theta_measurement - theta_estimate)

The full Hartley contact-aided InEKF belongs to a floating-base Lie-group state with IMU propagation and leg-contact observations. This repo teaches that transition but does not relabel a wrapped-angle EKF as the humanoid InEKF.

## Lin

Learned contact event:
- input: proprioceptive/history features,
- output: contact on/off,
- role: decide when contact-aided InEKF updates are admissible.

## Youm / NMN

The neural network supplies a learned body-velocity measurement in addition to contact information.

This gives the network more authority than a binary gate.

## InNKF

Physics filter first, learned correction second.

The paper uses a temporal network to predict posterior residual on the Lie group. The CartPole runtime includes a smaller trained residual MLP only to demonstrate the hybrid pattern.

## CoCo-InEKF

The important conceptual shift is:

    contact is not simply true/false
    measurement quality can be directional and continuous

The learned covariance controls how strongly each contact-derived measurement affects the filter.

## FOCUS

FOCUS asks a more estimator-specific question:

    "How reliable is this FK-derived velocity measurement right now?"

rather than merely:

    "Is the foot in contact?"

The CartPole Adaptive-R mode shows the mathematical location of this idea:

    reliability ↓
      → measurement covariance R ↑
      → Kalman gain K ↓
      → measurement correction ↓

The CartPole reliability model is heuristic, not a reproduction of FOCUS.
