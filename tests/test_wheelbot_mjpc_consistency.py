"""Verify native MJPC run receipts, task identity, and a short MuJoCo replay."""
from pathlib import Path
import argparse
import csv
import copy
import hashlib
import json
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/mjpc_wheelbot"))
import run


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def need(condition, message):
    if not condition:
        raise ValueError(message)


def checked_profile(profile, target_model, fingerprint, numerics, task_path):
    original = ROOT / "assets/wheelbot/contact_model.xml"
    need(profile["xmlSha256"] == sha(original), "profile does not match original XML")
    need(fingerprint["original_xml_sha256"] == sha(original), "original XML hash mismatch")
    need(fingerprint["plant_equal"], "runtime task changed the physical plant")
    base = mujoco.MjModel.from_xml_path(str(original))
    before, after = run.plant_snapshot(base), run.plant_snapshot(target_model)
    snapshot_sha = hashlib.sha256(json.dumps(after, sort_keys=True).encode()).hexdigest()
    original_sha = hashlib.sha256(json.dumps(before, sort_keys=True).encode()).hexdigest()
    need(fingerprint["original_snapshot_sha256"] == original_sha, "original numerical source hash mismatch")
    need(fingerprint["original_snapshot_sha256"] == fingerprint["numeric_plant_sha256"], "plant snapshot receipt hash mismatch")
    need(fingerprint["runtime_snapshot_sha256"] == snapshot_sha, "runtime numeric snapshot hash mismatch")
    need(fingerprint["runtime_xml_sha256"] == sha(task_path), "runtime task XML hash mismatch")
    comparable = json.loads(json.dumps(after))
    allowed = fingerprint["allowed_flags_scope"]
    if numerics == "reproducible":
        need(allowed == ["disableflags: mjDSBL_WARMSTART"], "unexpected reproducible flag scope")
        warmstart = int(mujoco.mjtDisableBit.mjDSBL_WARMSTART)
        need(int(target_model.opt.disableflags) == int(base.opt.disableflags) | warmstart, "reproducible profile changed unexpected disable flags")
        comparable["option"]["disableflags"] = before["option"]["disableflags"]
    else:
        need(allowed == [], "legacy profile changed plant flags")
    need(before == comparable, "run.plant_snapshot profile comparator found physical plant changes")
    gain_id = mujoco.mj_name2id(target_model, mujoco.mjtObj.mjOBJ_NUMERIC, "standing_K")
    if gain_id >= 0:
        np.testing.assert_array_equal(target_model.numeric("standing_K").data, np.asarray(profile["K"]).ravel())
    expected = profile["qref"] + profile["uref"]
    actual = np.r_[target_model.numeric("trim_q").data, target_model.numeric("trim_u").data]
    np.testing.assert_allclose(actual, expected, rtol=0, atol=0, err_msg="runtime trim numerics differ from profile")


def checked_predictions(predictions):
    times = np.asarray(predictions["time"], dtype=float)
    qpos = np.asarray(predictions["qpos"], dtype=float)
    qvel = np.asarray(predictions["qvel"], dtype=float)
    actions = np.asarray(predictions["actions"], dtype=float)
    recorded = np.asarray(predictions["recorded_actions"], dtype=float)
    need(times.shape == (6,) and qpos.shape == (6, 6) and qvel.shape == (6, 6), "predictions must contain six states")
    need(actions.shape == (5, 3) and recorded.shape == (5, 3), "predictions must contain five three-axis actions")
    need(all(np.isfinite(x).all() for x in (times, qpos, qvel, actions, recorded)), "nonfinite prediction")
    need(np.allclose(np.diff(times), .002, rtol=0, atol=1e-12), "prediction timestep mismatch")
    np.testing.assert_array_equal(actions, recorded, err_msg="predicted actions differ from recorded controls")
    return times, qpos, qvel, actions


def max_state_error(data, qpos, qvel):
    return float(max(np.max(np.abs(data.qpos - qpos)), np.max(np.abs(data.qvel - qvel))))


def replay(model, initial, predictions):
    times, qpos, qvel, actions = checked_predictions(predictions)
    state_spec = int(initial["state_spec"])
    need(state_spec == mujoco.mjtState.mjSTATE_INTEGRATION, "initial state spec is not mjSTATE_INTEGRATION")
    state = np.asarray(initial["state"], dtype=float)
    need(state.shape == (mujoco.mj_stateSize(model, state_spec),), "initial integration state has wrong size")
    need(np.isfinite(state).all(), "nonfinite initial state")
    need(initial["numerics"] in ("reproducible", "legacy"), "unknown recorded numerics mode")
    data = mujoco.MjData(model)
    mujoco.mj_setState(model, data, state, state_spec)
    need(abs(data.time - times[0]) < 1e-12, "initial state time differs from predictions")
    np.testing.assert_array_equal(data.qpos, qpos[0], err_msg="initial qpos mismatch")
    np.testing.assert_array_equal(data.qvel, qvel[0], err_msg="initial qvel mismatch")
    open_q, open_v = 0.0, 0.0
    for i, action in enumerate(actions):
        data.ctrl[:] = action
        mujoco.mj_step(model, data)
        open_q = max(open_q, float(np.max(np.abs(data.qpos - qpos[i + 1]))))
        open_v = max(open_v, float(np.max(np.abs(data.qvel - qvel[i + 1]))))

    one_q, one_v = 0.0, 0.0
    for i, action in enumerate(actions):
        cold = mujoco.MjData(model)
        cold.time = times[i]
        cold.qpos[:] = qpos[i]
        cold.qvel[:] = qvel[i]
        cold.ctrl[:] = action
        cold.qacc_warmstart[:] = 0
        mujoco.mj_forward(model, cold)
        mujoco.mj_step(model, cold)
        one_q = max(one_q, float(np.max(np.abs(cold.qpos - qpos[i + 1]))))
        one_v = max(one_v, float(np.max(np.abs(cold.qvel - qvel[i + 1]))))
    need(open_q < 1e-9 and open_v < 1e-8, f"continuous plan replay errors q={open_q:g}, v={open_v:g}")
    need(one_q < 1e-9 and one_v < 1e-8, f"cold one-step replay errors q={one_q:g}, v={one_v:g}")
    return {"prediction_qpos_max_error": open_q, "prediction_qvel_max_error": open_v,
            "cold_one_step_qpos_max_error": one_q, "cold_one_step_qvel_max_error": one_v}


def replay_trace(model, initial, trace_path):
    with trace_path.open() as stream:
        rows = list(csv.DictReader(stream))
    need(len(rows) > 1, "trace has too few rows")
    need(all(np.isfinite(float(v)) for row in rows for k,v in row.items() if k != "selectedpolicy"), "nonfinite trace")
    need(np.allclose(np.diff([float(r["time"]) for r in rows]), .002, rtol=0, atol=1e-10), "trace timestep mismatch")
    state_spec = int(initial["state_spec"])
    state = np.asarray(initial["state"], dtype=float)
    data = mujoco.MjData(model)
    mujoco.mj_setState(model, data, state, state_spec)
    np.testing.assert_array_equal(data.qpos, [float(rows[0][f"q{i}"]) for i in range(6)])
    np.testing.assert_array_equal(data.qvel, [float(rows[0][f"v{i}"]) for i in range(6)])
    q_error, v_error = 0.0, 0.0
    for index, (row, nxt) in enumerate(zip(rows, rows[1:])):
        need(abs(data.time-float(row["time"])) < 1e-10, f"trace time discontinuity at row {index}")
        data.ctrl[:] = [float(row[f"u{i}"]) for i in range(3)]
        mujoco.mj_step(model, data)
        q_error = max(q_error, float(np.max(np.abs(data.qpos-[float(nxt[f"q{i}"]) for i in range(6)]))))
        v_error = max(v_error, float(np.max(np.abs(data.qvel-[float(nxt[f"v{i}"]) for i in range(6)]))))
    need(q_error < 1e-9 and v_error < 1e-8, f"full CSV replay errors q={q_error:g}, v={v_error:g}")
    return {"trace_steps":len(rows)-1, "trace_qpos_max_error":q_error, "trace_qvel_max_error":v_error}


def run_receipt(path):
    receipt = json.loads(path.read_text())
    if "runs" in receipt:
        results = []
        for item in receipt["runs"]:
            child = ROOT / item["receipt"]
            need(sha(child) == item["receipt_sha256"], f"run receipt hash mismatch: {child}")
            results.append(check_run(child, json.loads(child.read_text())))
        return results
    return [check_run(path, receipt)]


def checked_planner(receipt, trace_steps=None):
    p = receipt["planner"]
    need(p["consistency_pass"] is True, "planner consistency failed")
    need(type(p["rejected_plans"]) is int and p["rejected_plans"] == 0, "planner rejected plans")
    need(type(p["iterations"]) is int, "invalid iteration count")
    need(receipt["physics_dt_s"] == .002 and receipt["replan_dt_s"] == .01, "recorded timestep mismatch")
    need(p["horizon_steps"] == 501 and p["numerics"] == "reproducible", "invalid verified horizon/numerics")
    need(np.isfinite(p["final_cost"]), "nonfinite final cost")
    need(p["nominal_native_max_error"] == max(p["position_max_error_m_or_rad"], p["velocity_max_error_m_s_or_rad_s"]), "inconsistent residual maxima")
    steps = p["executed_steps"]
    need(type(steps) is int and steps > 0, "invalid executed steps")
    need(type(p["verified_plans"]) is int and p["verified_plans"] > 0, "no verified plans")
    need(p["verified_plans"] == p["iterations"] == (steps + 4)//5, "inconsistent plan counts")
    need(steps == round(float(receipt["command"][4])/.002), "incomplete execution")
    if trace_steps is not None:
        need(steps == trace_steps, "recorded executed steps mismatch")
    for tolerance, bound, fields in [
        ("position_tolerance", 1e-9, ["position_max_error_m_or_rad", "execution_position_max_error_m_or_rad"]),
        ("velocity_tolerance", 1e-8, ["velocity_max_error_m_s_or_rad_s", "execution_velocity_max_error_m_s_or_rad_s", "nominal_native_max_error"]),
        ("action_tolerance_nm", 1e-12, ["policy_action_max_error_nm"]),
    ]:
        need(p[tolerance] == bound, "changed tolerance")
        for field in fields:
            need(np.isfinite(p[field]) and 0 <= p[field] <= bound, "invalid residual: " + field)


def synthetic_checks(receipt):
    checked_planner(receipt)
    mutations = [("consistency_pass", False), ("verified_plans", 0), ("verified_plans", receipt["planner"]["verified_plans"]+1),
                 ("rejected_plans", 1), ("position_max_error_m_or_rad", .01),
                 ("policy_action_max_error_nm", .001), ("executed_steps", 99),
                 ("velocity_max_error_m_s_or_rad_s", float("nan")), ("position_tolerance", .1)]
    for field, value in mutations:
        bad = copy.deepcopy(receipt); bad["planner"][field] = value
        try:
            checked_planner(bad)
        except (ValueError, AssertionError):
            continue
        raise AssertionError("forged planner field accepted: " + field)


def check_run(path, receipt):
    checked_planner(receipt)
    need(receipt.get("returncode") == 0, f"run did not succeed: {path}")
    adapter = ROOT / "scripts/mjpc_wheelbot/main.cc"
    if sha(adapter) != receipt["adapter_sha256"]:
        adapter = ROOT / "test-results/mjpc-adopt" / ("main-" + receipt["adapter_sha256"] + ".cc")
    need(adapter.is_file() and sha(adapter) == receipt["adapter_sha256"], "adapter source hash mismatch")
    build_path = ROOT / receipt["build_receipt"]
    build = json.loads(build_path.read_text())
    need(build["adapter_sha256"] == receipt["adapter_sha256"], "build receipt adapter mismatch")
    need(build["binary_sha256"] == receipt["binary_sha256"], "build receipt binary mismatch")
    library = Path(build["environment"]["mujoco"]) / "libmujoco.so.3.7.0"
    need(sha(library) == receipt["source"]["library_sha256"], "MuJoCo library hash mismatch")
    for name, expected in receipt["source"]["source_sha256"].items():
        source_path = Path(build["environment"]["source"]) / "mjpc" / name
        need(sha(source_path) == expected, f"upstream source hash mismatch: {name}")
    need(receipt["source"]["header_version"] == receipt["source"]["runtime_version"] == 3007000, "MuJoCo runtime version mismatch")
    task_path = ROOT / receipt.get("task_xml", receipt["command"][1])
    fingerprint = receipt.get("task_model", receipt.get("plant"))
    if fingerprint is None:
        fingerprint = receipt["plant"]
    need(sha(task_path) == fingerprint["runtime_xml_sha256"], "runtime task XML hash mismatch")
    model = mujoco.MjModel.from_xml_path(str(task_path))
    profile = json.loads((ROOT / "assets/wheelbot/contact_profile.json").read_text())
    numerics = receipt.get("numerics_profile", fingerprint["numerics_profile"])
    checked_profile(profile, model, fingerprint, numerics, task_path)
    artifacts = receipt["raw_artifacts"]
    initial_path = ROOT / artifacts["initial"]["path"]
    predictions_path = ROOT / artifacts["predictions"]["path"]
    need(sha(initial_path) == artifacts["initial"]["sha256"], "initial state artifact hash mismatch")
    need(sha(predictions_path) == artifacts["predictions"]["sha256"], "prediction artifact hash mismatch")
    initial, predictions = json.loads(initial_path.read_text()), json.loads(predictions_path.read_text())
    need(initial["numerics"] == numerics, "initial state numerics profile mismatch")
    for field in ("time", "qpos", "actions"):
        bad = copy.deepcopy(predictions)
        if field == "time": bad[field][1] += .001
        else: bad[field][0][0] = float("nan")
        try: checked_predictions(bad)
        except (ValueError, AssertionError): pass
        else: raise AssertionError("malformed prediction accepted: " + field)
    need(not receipt.get("profile_sha256") or receipt["profile_sha256"] == sha(ROOT / "assets/wheelbot/contact_profile.json"), "profile source hash mismatch")
    bad_initial = copy.deepcopy(initial)
    bad_initial["state"][1] += .01
    try: replay(model, bad_initial, predictions)
    except (ValueError, AssertionError): pass
    else: raise AssertionError("mutated initial source state accepted")
    errors = replay(model, initial, predictions)
    trace_path = ROOT / receipt["trace"]
    if "trace_sha256" in receipt:
        need(sha(trace_path) == receipt["trace_sha256"], "trace source hash mismatch")
    errors.update(replay_trace(model, initial, trace_path))
    checked_planner(receipt, errors["trace_steps"])
    synthetic_checks(receipt)

    mutated = json.loads(json.dumps(predictions))
    mutated["actions"][0][0] += 1e-6
    try:
        checked_predictions(mutated)
    except (ValueError, AssertionError):
        pass
    else:
        raise AssertionError("synthetic action mutation passed prediction validator")
    bad_profile = json.loads(json.dumps(profile))
    bad_profile["qref"][0] += 1e-6
    try:
        checked_profile(bad_profile, model, fingerprint, numerics, task_path)
    except (ValueError, AssertionError):
        pass
    else:
        raise AssertionError("synthetic profile mutation passed profile validator")
    return {"receipt": str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path), **errors}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("receipt", nargs="?", default="evidence/wheelbot_mjpc_consistency.json")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = {"schema": "wheelbot-mjpc-consistency/v1", "passed": True, "results": run_receipt(ROOT / args.receipt)}
    output = args.output
    if output:
        if not output.is_absolute():
            output = ROOT / output
    else:
        output = ROOT / "test-results/mjpc-consistency/report.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    for result in report["results"]:
        print(json.dumps(result, sort_keys=True))
    print(f"PASS: {len(report['results'])} MJPC consistency receipt(s); report {output.relative_to(ROOT) if output.is_relative_to(ROOT) else output}")


if __name__ == "__main__":
    main()
