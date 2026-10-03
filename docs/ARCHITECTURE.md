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

This is deliberate. A PID page and a full NMPC page should not secretly use different environments.

## Runtime state

State:

    x = [position, velocity, pole angle, pole angular velocity]

Direct measurements:

    y = [position, pole angle]

Velocity is therefore hidden from the raw sensor path and must be obtained by finite difference or an observer.

## Timing

- plant integration: 200 Hz
- control/observer tick: 50 Hz
- linear MPC horizon: 30 ticks
- centroidal-style reduced horizon: 32 ticks
- full nonlinear NMPC horizon: 30 ticks

## Truth boundary

Simulator truth is used for:
- rendering,
- error/evidence metrics,
- the explicit Truth/oracle baseline.

Normal controller modes receive the selected observer state estimate, not simulator truth.
