# Architecture

The lab keeps one plant and changes the control or estimation block around it.

    plant truth x
        ↓
      sensor y
        ↓
      observer
      x_hat, P
        ↓
     controller
        ↓
      force u
        ↓
      plant

The browser always renders the same:
- CartPole truth and estimate overlay,
- disturbance controls,
- state trace,
- control trace,
- controller/observer context graph.

This is deliberate. A PID page and a full NMPC page must not secretly use different plants.

## Runtime state

State:

    x = [position, velocity, pole angle, pole angular velocity]

Direct measurements:

    y = [position, pole angle]

Velocity is hidden from the raw sensor path and must be obtained by finite difference or an observer.

## Discrete-time ordering

After initialization from y_0, every 20 ms control step is ordered as:

    x_hat_k
       ↓
    controller
       ↓
      u_k
       ↓
    plant transition
       ↓
    x_(k+1)
       ↓
    sensor
       ↓
    y_(k+1)
       ↓
    observer predict/update with u_k
       ↓
    x_hat_(k+1)

This ordering matters. An earlier prototype incorrectly predicted to k+1 and then updated with y_k, which mixed timestamps. The audited implementation stores truth, measurement, and estimate at the same post-step timestamp.

## Timing

- plant integration: 200 Hz (four 5 ms substeps)
- controller / observer tick: 50 Hz (20 ms)
- linear MPC horizon: 30 ticks
- centroidal-style reduced horizon: 32 ticks
- full nonlinear NMPC horizon: 30 ticks

Reported solver milliseconds are runtime diagnostics on the current host/browser, not portable real-time guarantees.

## Linear model boundary

The explanatory continuous model is retained in the source, but runtime LQR/KF/linear MPC use the discrete Jacobian of the same 20 ms nonlinear plant transition at the upright equilibrium.

This avoids silently controlling one discretized model while simulating another.

## Sensor covariance boundary

Each scenario declares a Gaussian measurement standard deviation for position and angle. KF/EKF-family observers receive:

    R = diag(sigma_position^2, sigma_angle^2)

for that scenario.

The measurement-glitch scenario then adds an extra deterministic position outlier that is intentionally **not** encoded in baseline R. This makes it useful for demonstrating outlier down-weighting.

The estimator nonlinear bench is different: it starts the pole at 0.5 rad (28.6 deg) with very low sensor noise and no parameter mismatch. It is intended for a fixed LQR-controller KF-versus-EKF comparison with no external push, so nonlinear prediction error is isolated from controller-family and disturbance effects. It is intentionally excluded from the general controller × observer matrix.

Q is a tuned educational process covariance, not a hardware-identified parameter.

## Truth boundary

Simulator truth is used for:
- rendering,
- error/evidence metrics,
- the explicit Truth/oracle baseline.

Normal controller modes receive the selected observer state estimate, not simulator truth.

The audited oracle regression requires exactly zero estimator RMSE; any nonzero oracle RMSE indicates timestamp misalignment.

## Learned-output covariance boundary

The InNKF-style residual MLP corrects only the reported estimator output. It is not fed back into the base EKF.

Therefore:
- baseP = covariance maintained by the base EKF,
- corrected output P = not calibrated / unavailable.

The UI deliberately does not draw baseP as confidence bounds for the neural-corrected estimate.
