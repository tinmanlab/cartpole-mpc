"""Real HTTP + MuJoCo WASM smoke checks for the wheelbot teaching page."""
import functools
import http.server
import json
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

        # A pose probe must move the articulated pivots without starting physics.
        hip_before = page.evaluate("window.wheelbotLab.getState().truth[3]")
        page.locator("#pose").click()
        hip_after = page.evaluate("window.wheelbotLab.getState().truth[3]")
        assert abs(hip_after - hip_before) > 0.05
        assert "POSE PROBE" in page.locator("#status").inner_text()

        # Confirm the unsupported 3-input MPC boundary fails visibly, then return to LQR.
        rejected = page.evaluate("window.wheelbotLab.configureMode('mpc')")
        assert rejected["accepted"] is False and "NOT_YET_SUPPORTED" in rejected["status"]
        page.evaluate("window.wheelbotLab.configureMode('lqr_kf')")
        page.wait_for_function("window.wheelbotLab.getState().truth12?.length === 12")
        result = page.evaluate("window.wheelbotLab.run(20)")
        assert len(result["truth12"]) == 12 and result["steps"] == 20 and not result["failed"]
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
            "unsupportedMpcFailsClosed": True,
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
