# Wheelbot keyboard playground

The default `wheelbot.html` is a full-screen keyboard playground: A/D requests horizontal velocity; the normal flat-ground cap is ±0.2 m/s and holding Shift ramps that cap toward ±0.4 m/s, then ramps it back down after release. W/S requests ±0.06 m/s height velocity within 36–49 cm; Q/E changes in-plane pitch within ±0.17 rad. Space charges for up to one simulated second. From rest it selects one of three independently verified low-pose stationary schedules with about 3.1 / 5.7 / 7.9 cm wheel clearance; while moving it retains the existing calibrated moving-hop family and launch-velocity intent. These are local tested ranges, not global physical maxima. Pause, focus loss, hidden tabs and course changes cancel charge; R resets the state, not a get-up maneuver.

Courses add actual MuJoCo collision geometry to the canonical robot and floor. The controller uses a known static terrain map. Compiling and displaying a course does not prove traversal. Actual keyboard, flight, focus, layout and wall-clock acceptance runs only in normal CI via `tests/test_wheelbot_game_browser.py --url URL`; local syntax checks do not establish browser acceptance.

## Game control boundaries

Releasing Space never overwrites position, velocity or the observer. The jump reference is translated using the measured launch velocity, with wheel rolling rate and joint damping compensation. The horizontal keys are retained and become ground velocity commands again after landing. This preserves motion; it does not claim arbitrary horizontal acceleration is sustainable in free flight without an external force. The base can briefly recoil during landing as the leg reconfigures. Moving-jump acceptance therefore checks preserved launch state, forward airborne total-COM movement and resumed ground velocity, not a never-decreasing base velocity through impact. The measured minimum base velocity is retained separately. Charge is based on simulated held duration, capped at one second, independent of keyboard repeat.

The two source hops share an approximately 39.25 cm entry and a roughly 35.47 cm minimum hip-axis height. Higher charge therefore does not imply proportionally deeper crouching in this implementation; calibrated push-off/flight scheduling changes the requested clearance. W/S or Q/E may put the robot outside the checked jump-entry neighborhood. A release then reports the reason instead of silently moving to another pose or pretending to jump.

The flat-ground measured envelope covers the original steady ±0.2 m/s motion plus the progressive Shift speed-cap path to ±0.4 m/s. Smooth ramp and uneven courses use finite-radius map preview, terrain-relative reference height, a short-window measurement-rate correction near contact transitions, and terrain-only wheel-rate damping; the checked first traversal remains inside the existing physical failure gates. Sharp vertical box steps are different hybrid contact modes. The current rolling controller therefore declares them outside its admitted domain and stops before first contact rather than driving into the face and falling. A real step-climb maneuver or hybrid MPC is a separate controller task; no threshold or torque limit is relaxed to fake traversal. Failed admitted control still disengages all motors and invalidates its estimate while gravity and collision integration continue. Press R for an explicit reset.

Jump data is optional in the current playground: missing, malformed, baseline-mismatched, or runtime-mismatched jump data disables Space/jump while ordinary A/D/W/S/Q/E balancing remains available. Terrain reference context uses finite-radius wheel support derived from the exact course geoms and the compiled wheel radius; low steps and ramp corners are represented before the wheel centre crosses an edge, while a vertical rise larger than one wheel radius remains a blocking obstacle. These checks improve reference/contact consistency but do not certify universal terrain traversal.

## Preserved path lesson

The following describes `wheelbot-paths.html`, which retains the existing path application and its numerical/browser regressions.

[Path lesson](https://tinmanlab.github.io/cartpole-mpc/wheelbot-paths.html) · [CartPole](../index.html)

## One model and an explicit runtime version

The page loads only `assets/wheelbot/live_model.xml`: a planar wheelbot with a 170 × 140 × 150 mm, 1 kg box torso. The uniform-box inertia is `mass / 3 * (halfside² + halfside²)` on each diagonal. The declared torso COM is 20 mm above the hip axis. The original links, wheel and contact model remain; motor ceilings are 16 / 16 / 1.7 N·m, physics advances every 2 ms and feedback every 10 ms. These are model assumptions, not identified hardware speed, thermal or battery limits. Historical robot assets are not selectable on this page.

Native and browser MuJoCo are pinned to 3.15.0. The package lock, native requirements, vendored upstream files and manifest agree with the loaded engine. Base gains, the 25-pose atlas and selected jump schedules were regenerated with that engine. Historical receipts are not relabelled. The viewer displays the actual engine and generation versions; incompatible optional jump data disables jumping without disabling ordinary balance.

## Click, draw and follow

Choose **몸통** or **바퀴 중심**, click a target or draw an ordered path and release. Base means the torso origin at the hip axis, not COM. Wheel means wheel centre; in standing contact it is approximately 0.05 m above the existing floor. Only planar position and pitch are represented, not roll/yaw.

Grey dashed paths show requests, green paths show accepted candidates and blue paths show actual motion. The gold body shows the reference currently supplied to the controller. Sliders use the same admission path. Requests never teleport the robot or reset its Kalman estimate. Cancelling removes future reference commands while balance continues; Reset is explicit state initialization, not physical get-up.

The planner compresses near-collinear points within 3 mm while retaining corners, then uses quintic reference segments. It includes reference rates for body height/pitch and the induced hip/knee motion. Wheel rolling rate is calculated from MuJoCo kinematics. Candidate coordinate rates of 0.24 m/s, 0.06 m/s and 0.24 rad/s are software design values, not physical maxima.

The geometric atlas projection is only a candidate. Up to three retimings are checked with the official nonlinear plant and noisy estimator, including motor, joint, contact, self-collision and tracking limits. Preview has separate MuJoCo data and does not alter live state, force history, step counts or measurement noise. It yields periodically so simulation can continue, retries once when the actual state invalidates its prediction, and has a five-second wall-time budget. It can reject long or complicated requests. No globally closest path or minimum-time solution is claimed.

Rejected replacement requests retain braking/balance rather than silently executing an unchecked trajectory. During replacement, a bounded 0.20-second reference brake may replace the old future path. The checked trajectory is followed by up to 2.5 seconds of settling, with final limits evaluated over the last 0.5 seconds. The viewer reports this duration separately.

## Small jumps from a checked current state

Choose the explicit **낮은 자세로 이동** target to move under normal ground control toward 0.3925 m and zero pitch. It is not a hidden preparation action inside the jump command. Jump readiness is calculated from the current estimate and shown with the missing admission conditions.

Once ready, select **바퀴 중심 → 작은 점프** and click a target. The planner chooses the nearest eligible wheel-apex candidate from a finite seven-primitive family. The accepted apex, existing-floor landing and path are shown separately from the request. Jump admission preserves the actual state and estimator; a physical crouch/push-off begins from that state. No upward external force, alternate model or pose overwrite is used.

The checked neighborhood is narrow, around 0.3925/0.425 m standing heights, near-zero pitch, small joint errors and low rates. The default approximately 0.476 m stance is outside it. The small hops have approximately 2–5 cm tyre clearance. Translation of a reference in world x is not proof of arbitrary left/right relative jumping. Base-apex targets, freely drawn flight paths and complete floor-to-standing recovery are not established and must not be represented as available.

A 600-command run is not, by itself, jump success. The completion check requires observed flight, contact landing, at least 2 cm clearance and terminal base errors within 2 cm / 5 mm / 0.03 rad with rates at most 0.3. An unmet outcome is reported separately while physically admissible balance can continue. Physical violations disengage the motors. The 0.5 m/s first-landing tangential-slip ceiling is an explicit simulation benchmark assumption, not a hardware-derived limit.

Wheel acceleration produces reaction torque about the same planar y axis, not gyroscopic cross-axis stabilization. The displayed spoke uses the physical absolute wheel angle. Small-jump design considers wheel-rate and actuator-work penalties; it does not freeze wheel state or merely change the drawing. Lower spin on these smaller hops is not an equal-height optimality or energy-efficiency proof.

## Measurements and timing

Five noisy pose channels plus a noisy wheel encoder-rate channel feed the observer. Feedback uses the estimate; true state and contact results support visualization and evaluation. They are not real sensor recordings. The viewer's reproducible measurement seed is 7; that is a development scenario, not a robustness claim.

Push, gust and twist are finite physical inputs with light/medium/strong settings, bounded by 4 N or 0.1 N·m. Strong pulses may leave the tested control domain. Reset must never be described as recovery.

Pure control, plant, planning, rendering and complete-loop timing are separate. Manual batch stepping and pause are not wall-clock playback. Detailed metrics are serialized only while their panel is open. Native/Node throughput is not browser FPS, real-time certification or a hardware deadline guarantee.

## Reproduction and remaining performance gap

The engine-only release recorded 1/8 on the original 1.5-second response benchmark. The responsive-tracking revision keeps the same plant, Q/R, sensor noise, torque limits and task requests, but plans affine feedforward and time-varying error feedback from the supplied local dynamics. It no longer treats each moving target sample as an unrelated static equilibrium. The terminal cost-to-go uses the existing posture design. Every candidate still passes the nonlinear contact/limit/settling preview before execution. The original unreachable height remains a projected request, not an original-target success; single-step, path and projected-target results remain separate. Run the strengthened `--require-feasible-fast` check for sustained feasible-step settling and overshoot rather than inferring speed from eventual stability.

Version gates: `python tests/test_runtime_versions.py` and `node tests/test_runtime_versions.mjs`. Ground tests: `node tests/test_wheelbot_box_paths.mjs`, `node tests/test_wheelbot_paths_runtime.mjs`, then the native box/pose parity tests. Append `--require-fast` to retain the unmet fast benchmark as an explicit failure.

Jump tests: `python tests/test_wheelbot_target_jump.py`, `node tests/test_wheelbot_target_jump.mjs`, `node tests/test_wheelbot_target_jump_actions.mjs` and `node tests/test_wheelbot_jump_completion.mjs`. These separate primitive parity, action integration and the completion predicate. Completion rejects missing, wrong-length, sparse and nonfinite state/reference arrays before checking physical outcomes; malformed data cannot count as success.

Normal CI runs `tests/test_wheelbot_paths_browser.py --url URL` against the actual page. Its positive jump gate requires the explicit low target, bounded ordinary settling and a real pointer-triggered jump without an intervening reset. Rejection is not an alternative pass. Negative version, unsupported target and optional-data cases are separate. Actual browser evidence is required before viewer acceptance; syntax compilation alone is insufficient.

The original Upkie parameter source and licence remain in `assets/wheelbot/source/`. MuJoCo supplies the dynamics; this is a model-specific educational lab, not a universal robot importer or registered WebMCP interface.

### Runtime work removed without changing the physical integration

Prepared paths cache interpolated dynamics, reference states and gains outside the fast loop. The runtime applies the planned feedforward plus state-estimation-error correction; no optimizer runs per control step. Geometric drawing uses official `mj_kinematics`, and contact/geometric telemetry uses the official position stage instead of repeating full acceleration/contact-force solves. Actual `mj_step`, force queries, joint limits and contact parameters are unchanged. Interleaved old/new replay checks found no numerical differences over 5,000 integration steps. Algebraic correctness is independently checked by `tests/test_wheelbot_affine_reference.py` using SciPy. Local runtime timings vary with host load and are not hardware/WCET guarantees.
