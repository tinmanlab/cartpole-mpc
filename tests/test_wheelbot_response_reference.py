"""Independent response-profile and raw MPC trace receipt checks."""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.linalg import eigvals, norm, solve_discrete_are


ROOT = Path(__file__).resolve().parents[1]
BASELINE_PATH = ROOT / "assets/wheelbot/profile.json"
RESPONSE_PATH = ROOT / "assets/wheelbot/response_profile.json"
RAW_PATH = ROOT / "test-results/wheelbot_mpc_response_traces.json"
SUMMARY_PATH = ROOT / "evidence/wheelbot_mpc_response_validation.json"
MANIFEST_PATH = ROOT / "tests/fixtures/wheelbot_cases.json"

baseline = json.loads(BASELINE_PATH.read_text())
profile = json.loads(RESPONSE_PATH.read_text())
manifest = json.loads(MANIFEST_PATH.read_text())
raw_receipt = json.loads(RAW_PATH.read_text())
summary = json.loads(SUMMARY_PATH.read_text())
baseline_sha256 = hashlib.sha256(BASELINE_PATH.read_bytes()).hexdigest()
response_sha256 = hashlib.sha256(RESPONSE_PATH.read_bytes()).hexdigest()
assert baseline_sha256 == "d54d98ec5b18d89bdf4043a408ea49d7636edcb9c29301b88f446cda9a968e77"

# The response profile is a narrow cost change. Every other baseline datum,
# including the model and observer data, must remain byte-for-value equivalent.
allowed_changes = {
    "Q", "P", "K", "dareNormalizedResidual", "closedLoopRadius",
    "responseDesign",
}
assert {k: v for k, v in profile.items() if k not in allowed_changes} == {
    k: v for k, v in baseline.items() if k not in allowed_changes
}
design = profile["responseDesign"]
assert design["changedPreference"] == "Qc[0,0] only"
assert design["method"] == "scipy.optimize.brentq in log10 factor, fixed bracket [0,4]"
assert design["stepM"] == 0.03
assert design["targetM"] == 0.01
assert design["tailSamples"] == [250, 300]
assert design["sourceProfileSha256"] == baseline_sha256

A = np.asarray(profile["A"], dtype=float)
B = np.asarray(profile["B"], dtype=float)
Q = np.asarray(profile["Q"], dtype=float)
R = np.asarray(profile["R"], dtype=float)
P_saved = np.asarray(profile["P"], dtype=float)
K_saved = np.asarray(profile["K"], dtype=float)
P = solve_discrete_are(A, B, Q, R)
K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
np.testing.assert_allclose(P_saved, P, rtol=1e-10, atol=1e-10)
np.testing.assert_allclose(K_saved, K, rtol=1e-10, atol=1e-10)
assert np.max(np.abs(Q - np.asarray(baseline["Q"], dtype=float)
                      - np.diag([Q[0, 0] - baseline["Q"][0][0]]
                                + [0.0] * (Q.shape[0] - 1)))) < 1e-12
multiplier = float(design["factor"])
assert multiplier > 0
assert abs(Q[0, 0] / baseline["Q"][0][0] - multiplier) < 1e-10

closed = A - B @ K
riccati = A.T @ P @ A - P - A.T @ P @ B @ K + Q
normalized_residual = float(norm(riccati, np.inf) / max(1.0, norm(P, np.inf)))
radius = float(max(abs(eigvals(closed))))
assert abs(float(profile["dareNormalizedResidual"]) - normalized_residual) < 1e-12
assert normalized_residual <= 1e-10
assert abs(float(profile["closedLoopRadius"]) - radius) < 1e-10
assert radius < 1.0

# Rebuild the nominal full-state 3 cm position-step prediction directly from
# the DARE result; the maximum error is measured over samples 250 through 300.
target = np.zeros(A.shape[0])
target[0] = 0.03
state = np.zeros(A.shape[0])
errors = []
for sample in range(301):
    if 250 <= sample <= 300:
        errors.append(abs(float(state[0] - target[0])))
    state = closed @ state + B @ K @ target
max_error = max(errors)
root_residual = max_error - 0.01
assert max_error <= 0.01 + 1e-10, max_error
assert abs(root_residual) <= 1e-10, root_residual

assert set(raw_receipt) >= {"profileSha256", "rows"}
assert set(summary) >= {"profileSha256", "rows"}
assert raw_receipt["profileSha256"] == summary["profileSha256"]
assert raw_receipt["profileSha256"] == {
    "baseline": baseline_sha256,
    "response": response_sha256,
}
raw_rows = raw_receipt["rows"]
summary_rows = summary["rows"]
assert len(raw_rows) == len(summary_rows) == 48
key_fields = ("profile", "mode", "name", "seed")
key = lambda row: tuple(row[field] for field in key_fields)
raw_by_key = {key(row): row for row in raw_rows}
summary_by_key = {key(row): row for row in summary_rows}
assert len(raw_by_key) == len(raw_rows), "duplicate raw trial key"
assert len(summary_by_key) == len(summary_rows), "duplicate summary trial key"
assert raw_by_key.keys() == summary_by_key.keys()
assert {k[0] for k in raw_by_key} == {"baseline", "response"}
assert {k[1] for k in raw_by_key} == {"lqr_kf", "mpc_kf"}
assert {k[2] for k in raw_by_key} == {c["name"] for c in manifest["cases"]}
assert {k[3] for k in raw_by_key} == {101, 211, 307}

controlled = baseline["controlledIndices"]
qref = baseline["qref"]
units = ["m", "m", "rad", "rad", "rad", "m/s", "m/s", "rad/s",
         "rad/s", "rad/s", "rad/s"]
dt = float(baseline["controlDt"])


def close(actual, expected, label, *, atol=1e-10, rtol=1e-10):
    np.testing.assert_allclose(actual, expected, atol=atol, rtol=rtol,
                               err_msg=label)


for trial_key, raw in raw_by_key.items():
    row = summary_by_key[trial_key]
    trace = raw["trace"]
    case = next(c for c in manifest["cases"] if c["name"] == raw["name"])
    assert len(trace) <= manifest["steps"]
    expected_initial = [*qref, 0, 0, 0, 0, 0, 0]
    expected_initial[2] += case["pitchOffset"]
    close(raw["initial"], expected_initial, f"{trial_key} initial state")
    assert raw["rejection"] is None or isinstance(raw["rejection"], str)
    if trace:
        truths = np.asarray([snap["truth"] for snap in trace], dtype=float)
        estimates = np.asarray([snap["estimate"] for snap in trace], dtype=float)
        assert truths.ndim == 2 and truths.shape[1] == 12
        assert estimates.shape == (len(trace), len(controlled))
        final = trace[-1]
        steps = int(final["steps"])
        assert steps == len(trace)
    else:
        truths = np.empty((0, 12))
        estimates = np.empty((0, len(controlled)))
        final = None
        steps = 0

    failed = any(bool(snap["failed"]) for snap in trace)
    completed = steps == manifest["steps"] and not failed and raw["rejection"] is None
    tail = trace[-manifest["tailSteps"]:]
    tail_position = max((abs(float(s["truth"][0] - qref[0] - case["goal"]))
                         for s in tail), default=None)
    tail_pitch = max((abs(float(s["truth"][2] - qref[2])) for s in tail),
                     default=None)
    task_passed = completed and len(tail) == manifest["tailSteps"] and all(
        abs(s["truth"][0] - qref[0] - case["goal"]) < manifest["positionTolerance"]
        and abs(s["truth"][2] - qref[2]) < manifest["pitchTolerance"]
        for s in tail
    )
    rmse = (np.sqrt(np.mean((truths[:, controlled] - estimates) ** 2, axis=0))
            if trace else [None]*len(controlled))
    saturations = sum(bool(s["last"]["saturated"]) for s in trace)
    contact_loss = sum(not s["last"]["contact"]["wheelContacts"] for s in trace)
    max_slip = max((abs(float(s["last"]["contact"]["slip"])) for s in trace),
                   default=None)
    torque_rms = np.sqrt(np.mean(
        np.asarray([s["last"]["u"] for s in trace], dtype=float) ** 2, axis=0
    )) if trace else [None]*3
    slew = np.asarray([
        (np.asarray(s["last"]["u"], dtype=float)
         - (np.asarray(trace[i - 1]["last"]["u"], dtype=float)
            if i else np.asarray(baseline["uref"], dtype=float))) / dt
        for i, s in enumerate(trace)
    ])
    slew_rms = np.sqrt(np.mean(slew ** 2, axis=0)) if trace else [None]*3
    solver_max_ms = max((float(s["last"]["solveMs"]) for s in trace), default=None) if raw["mode"] == "mpc_kf" else None
    active_samples = saturations
    same_estimate_difference = max((
        abs(float(u - lqr))
        for s in trace
        for u, lqr in zip(s["last"]["u"], s["last"]["sameEstimateLqrU"])
    ), default=None)
    expected = {
        "completed": completed,
        "taskPassed": task_passed,
        "steps": steps,
        "qpRejected": raw["rejection"] is not None,
        "rejection": raw["rejection"],
        "physicalEnvelopeFailure": failed,
        "contactLoss": contact_loss,
        "saturations": saturations,
        "units": units,
        "tailPositionMax": tail_position,
        "tailPitchMax": tail_pitch,
        "maxSlip": max_slip,
        "torqueRmsNm": torque_rms,
        "slewRmsNmPerSecond": slew_rms,
        "elapsedMs": float(raw["elapsedMs"]),
        "appliedTorqueAtLimitSamples": active_samples,
        "predictedConstraintActiveSamples": sum(bool(s["last"].get("forecastConstraintActive")) for s in trace) if raw["mode"]=="mpc_kf" else None,
        "maxSameEstimateLqrDifferenceNm": same_estimate_difference,
        "solverMaxMs": solver_max_ms,
    }
    for field, expected_value in expected.items():
        assert field in row, (trial_key, "missing metric", field)
        if (isinstance(expected_value, (bool, str)) or expected_value is None
                or field == "units"):
            assert row[field] == expected_value, (trial_key, field, row[field], expected_value)
        elif isinstance(expected_value, list) and all(v is None for v in expected_value):
            assert row[field] == expected_value
        else:
            close(row[field], expected_value, f"{trial_key} {field}")
    if trace:close(row["estimationRmseByState"], rmse, f"{trial_key} estimation RMSE")
    else:assert row["estimationRmseByState"]==[None]*len(controlled)
    if trace:
        close(row["final"], truths[-1], f"{trial_key} final state")
    else:
        close(row["final"], raw["initial"], f"{trial_key} unchanged zero-step state")

print(json.dumps({
    "schema": "wheelbot-response-independent-reference/v1",
    "trials": len(raw_rows),
    "responseMultiplier": multiplier,
    "responseMaxErrorM": max_error,
    "rootResidualM": root_residual,
    "normalizedDareResidual": normalized_residual,
    "closedLoopRadius": radius,
    "passed": True,
}, indent=2))
