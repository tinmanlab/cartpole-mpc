"""Real HTTP + MuJoCo WASM smoke checks for the wheelbot teaching page."""
import functools
import hashlib
import http.server
import json
import math
import threading
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]


class Quiet(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


server = http.server.ThreadingHTTPServer(
    ("127.0.0.1", 0), functools.partial(Quiet, directory=str(ROOT))
)
threading.Thread(target=server.serve_forever, daemon=True).start()
try:
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page(viewport={"width": 1440, "height": 1000})
        errors = []
        page.on("pageerror", lambda error: errors.append(str(error)))
        response = page.goto(
            f"http://127.0.0.1:{server.server_port}/wheelbot.html",
            wait_until="networkidle",
        )
        assert response.status == 200, "Wheelbot page missing"
        base_profile = json.loads((ROOT / "assets/wheelbot/profile.json").read_text())
        response_profile = json.loads((ROOT / "assets/wheelbot/response_profile.json").read_text())
        response_design = response_profile["responseDesign"]
        assert response_design["sourceProfileSha256"] == hashlib.sha256(
            (ROOT / "assets/wheelbot/profile.json").read_bytes()
        ).hexdigest()
        assert response_design["changedPreference"] == "Qc[0,0] only"
        assert response_design["factor"] > 0
        for key in ("A", "B", "R", "Qe", "Re", "measurementSigma", "qref", "uref", "limitsNm"):
            assert response_profile[key] == base_profile[key], f"response design changed {key}"
        assert response_profile["Q"][0][0] != base_profile["Q"][0][0]
        assert all(
            response_profile["Q"][i][j] == base_profile["Q"][i][j]
            for i in range(11) for j in range(11) if (i, j) != (0, 0)
        ), "response design may change only Qc[0,0]"
        assert math.isclose(
            response_profile["Q"][0][0] / base_profile["Q"][0][0],
            response_design["factor"], rel_tol=1e-12,
        )
        # Fetched response metadata must match the bytes actually loaded, not
        # merely carry the same XML hash or assert a Qx-only change.
        for mutation in ('source-hash', 'observer-noise'):
            served=[]
            # Playwright supplies (route, request). Keep the frozen mutation
            # keyword-only so Request cannot replace it.
            def invalid_response(route, *, mutation=mutation):
                bad=json.loads(json.dumps(response_profile))
                if mutation=='source-hash':
                    bad['responseDesign']['sourceProfileSha256']='0'*64
                else:
                    bad['measurementSigma'][0]*=2
                served.append({'mutation':mutation,'url':route.request.url,'source':bad['responseDesign']['sourceProfileSha256'],'noise':bad['measurementSigma'][0]})
                route.fulfill(status=200,content_type='application/json',body=json.dumps(bad))
            page.route('**/assets/wheelbot/response_profile.json',invalid_response)
            page.reload(wait_until='networkidle')
            page.wait_for_function('window.wheelbotLab?.ready',timeout=30000)
            assert served and all(x['mutation']==mutation for x in served), {'routeNotExercised':mutation,'served':served}
            diagnostics=page.evaluate("() => ({ready:window.wheelbotLab.ready,optionDisabled:document.querySelector('#design option[value=response]').disabled,info:document.querySelector('#design-info').textContent,status:document.querySelector('#status').textContent,design:window.wheelbotLab.getDesign()})")
            # The live DOM reported disabled=True while Playwright's helper
            # returned False for this option. Check its actual property and
            # the application's rejection path, not the parent select state.
            assert diagnostics['optionDisabled'] is True, {'mutation':mutation,'served':served,'browser':diagnostics}
            refused=page.evaluate("window.wheelbotLab.selectDesign('response')")
            assert refused['accepted'] is False and page.locator('#design').input_value()=='baseline', refused
            assert 'unavailable' in page.locator('#design-info').inner_text().lower()
            baseline_run=page.evaluate('window.wheelbotLab.run(20)')
            assert baseline_run['steps']==20 and not baseline_run['failed']
            assert page.evaluate('window.wheelbotLab.getDesign().selection')=='baseline'
            page.unroute('**/assets/wheelbot/response_profile.json',invalid_response)
        # Hash mismatch and malformed XML both fail closed before controls activate.
        def stale_profile(route):
            bad = dict(base_profile)
            bad["assetSha256"] = "0" * 64
            route.fulfill(status=200, content_type="application/json", body=json.dumps(bad))
        page.route("**/assets/wheelbot/profile.json", stale_profile)
        page.reload(wait_until="networkidle")
        page.wait_for_function("document.querySelector('#status').textContent.includes('REJECTED')")
        assert page.locator("#play").is_disabled(), "stale profile must not enable physics"
        page.unroute("**/assets/wheelbot/profile.json", stale_profile)
        def malformed_xml(route):
            route.fulfill(status=200, content_type="application/xml", body="<mujoco")
        page.route("**/assets/wheelbot/wheelbot.xml", malformed_xml)
        page.reload(wait_until="networkidle")
        page.wait_for_function("document.querySelector('#status').textContent.includes('REJECTED')")
        assert page.locator("#play").is_disabled(), "invalid model must not enable physics"
        page.unroute("**/assets/wheelbot/wheelbot.xml", malformed_xml)
        page.reload(wait_until="networkidle")
        page.wait_for_function("window.wheelbotLab?.ready", timeout=30000)
        before = page.evaluate("window.wheelbotLab.getState()")
        assert before["truth12"] and len(before["truth12"]) == 12, before
        assert before["steps"] == 0 and before["physics"]["backend"] == "mujoco-wasm"
        assert page.locator("#view").count() == 1
        assert "MuJoCo WASM" in page.locator("#status").inner_text()
        assert page.locator("#design").input_value() == "baseline"
        assert "original control" in page.locator("#design-info").inner_text().lower()
        assert "model-designed" in page.locator("#design-info").inner_text().lower()
        baseline_estimate = before["estimate"]

        # A pose probe must move the articulated pivots without starting physics.
        hip_before = page.evaluate("window.wheelbotLab.getState().truth[3]")
        page.locator("#pose").click()
        hip_after = page.evaluate("window.wheelbotLab.getState().truth[3]")
        assert abs(hip_after - hip_before) > 0.05
        assert "POSE PROBE" in page.locator("#status").inner_text()

        # An unsupported request stops an already-running trial.
        stopped = page.evaluate("""() => {
          window.wheelbotLab.configureMode('lqr_kf');
          document.querySelector('#play').click();
          const before = window.wheelbotLab.getState();
          const rejected = window.wheelbotLab.configureMode('nmpc');
          const after = window.wheelbotLab.run(1);
          return {before,after,rejected};
        }""")
        assert stopped['before']['playing'] and not stopped['after']['playing']
        assert stopped['after']['truth']==stopped['before']['truth']
        assert not stopped['rejected']['accepted'] and stopped['after']['steps']==0

        # Future nonlinear MPC is rejected without changing the active selection.
        rejected = page.evaluate("window.wheelbotLab.configureMode('nmpc')")
        assert rejected["accepted"] is False and "NOT_YET_SUPPORTED" in rejected["status"]
        assert rejected["mode"] == "lqr_kf", "rejected mode must not corrupt the selection"

        # Both feedback modes use the same seeded localization/KF estimator.
        mpc_selected = page.evaluate("window.wheelbotLab.configureMode('mpc_kf')")
        assert mpc_selected["accepted"] and mpc_selected["mode"] == "mpc_kf"
        mpc_initial = page.evaluate("window.wheelbotLab.getState()")
        lqr_selected = page.evaluate("window.wheelbotLab.configureMode('lqr_kf')")
        assert lqr_selected["accepted"] and lqr_selected["mode"] == "lqr_kf"
        lqr_initial = page.evaluate("window.wheelbotLab.getState()")
        assert mpc_initial["estimate"] == lqr_initial["estimate"]
        assert len(mpc_initial["estimate"]) == 11

        # Exercise both controller loops against the live MuJoCo-backed trial.
        page.evaluate("window.wheelbotLab.configureMode('mpc_kf')")
        mpc_result = page.evaluate("window.wheelbotLab.run(20)")
        assert len(mpc_result["truth12"]) == 12 and mpc_result["steps"] == 20 and not mpc_result["failed"]
        assert len(mpc_result["last"]["u"]) == 3, mpc_result["last"]
        assert mpc_result["last"]["solver"]
        assert len(mpc_result["last"]["forecastActiveCountsByMotor"]) == 3
        for key in ("kktResidual", "primalResidual", "solveMs"):
            assert isinstance(mpc_result["last"][key], (int, float))
            assert math.isfinite(mpc_result["last"][key]) and mpc_result["last"][key] >= 0
        assert all(abs(torque) <= limit + 1e-6 for torque, limit in zip(mpc_result["last"]["u"], (16, 16, 1.7)))

        lqr_result = page.evaluate("window.wheelbotLab.configureMode('lqr_kf'); window.wheelbotLab.run(20)")
        assert len(lqr_result["truth12"]) == 12 and lqr_result["steps"] == 20 and not lqr_result["failed"]
        assert len(lqr_result["estimate"]) == 11

        # The response design changes only the x-state preference. Selection
        # reconstructs a fresh seeded trial while preserving controller/goal.
        page.locator("#design").select_option("response")
        response_initial = page.evaluate("window.wheelbotLab.getState()")
        response_metadata = page.evaluate("window.wheelbotLab.getDesign()")
        assert response_metadata["selection"] == "response"
        assert response_metadata["sourceProfileSha256"] == response_design["sourceProfileSha256"]
        assert response_metadata["changedPreference"] == "Qc[0,0] only"
        assert response_design["sourceProfileSha256"] in page.locator("#design-info").inner_text()
        assert response_initial["estimate"] == baseline_estimate, "same seeded sensor noise should reproduce the KF initial estimate"
        assert response_initial["steps"] == 0
        response_lqr = page.evaluate("window.wheelbotLab.run(20)")
        assert response_lqr["steps"] == 20 and not response_lqr["failed"]
        response_mpc = page.evaluate("window.wheelbotLab.configureMode('mpc_kf'); window.wheelbotLab.run(20)")
        assert response_mpc["steps"] == 20 and not response_mpc["failed"]
        assert response_mpc["last"]["solver"]
        page.locator("#design").select_option("baseline")
        baseline_reset = page.evaluate("window.wheelbotLab.getState()")
        assert baseline_reset["steps"] == 0 and baseline_reset["estimate"] == baseline_estimate
        page.locator("#design").select_option("response")
        assert page.evaluate("window.wheelbotLab.getState().steps") == 0

        # Reset restarts the selected response-profile MPC trial while preserving all selections.
        page.locator("#goal").select_option("0.03")
        page.locator("#mode").select_option("mpc_kf")
        page.evaluate("window.wheelbotLab.run(5)")
        page.locator("#reset").click()
        reset_state = page.evaluate("window.wheelbotLab.getState()")
        assert page.locator("#design").input_value() == "response"
        assert page.locator("#mode").input_value() == "mpc_kf"
        assert page.locator("#goal").input_value() == "0.03"
        assert reset_state["steps"] == 0 and len(reset_state["estimate"]) == 11
        assert "Solver" in page.locator("#solver").inner_text()

        result = page.evaluate("window.wheelbotLab.run(20)")
        assert result["steps"] == 20 and not result["failed"]
        assert page.locator("#torques tr").count() == 3
        solver_text = page.locator("#solver").inner_text()
        assert all(label in solver_text for label in ("Solver", "KKT residual", "primal residual", "solve"))
        assert "contact" in page.locator("#contact").inner_text().lower()
        assert "m/s" in page.locator("#contact").inner_text()
        assert page.locator("#estimate tr").count() == 11
        page.screenshot(path=str(ROOT / "evidence/wheelbot_browser.png"), full_page=True)
        assert page.evaluate("document.documentElement.scrollWidth-innerWidth") <= 1
        assert not errors, errors
        evidence = {
            "schema": "wheelbot-browser/v1",
            "actualWasm": True,
            "stateDimension": len(result["truth12"]),
            "poseProbeArticulates": True,
            "unsupportedNmpcFailsClosed": True,
            "unsupportedRequestStopsExistingTrial": True,
            "mpcAndLqrRun": True,
            "sharedEstimatorSeed": True,
            "mpcSolverMetrics": True,
            "resetPreservesSelections": True,
            "baselineResponseDesigns": True,
            "responseChangesQxOnly": True,
            "fetchedResponseBoundToBaseline": True,
            "mismatchedResponsePreservesBaseline": True,
            "sharedSensorNoise": True,
            "responseLqrAndMpcRun": True,
            "staleProfileFailsClosed": True,
            "invalidModelFailsClosed": True,
            "steps": result["steps"],
            "pageErrors": errors,
            "passed": True,
            "publicDeployed": False,
        }
        (ROOT / "evidence/wheelbot_browser.json").write_text(
            json.dumps(evidence, indent=2) + "\n"
        )
        browser.close()
        print(json.dumps(evidence))
finally:
    server.shutdown()
