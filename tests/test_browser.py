from pathlib import Path
import asyncio, json
from playwright.async_api import async_playwright

ROOT = Path(__file__).resolve().parents[1]
HTML = (ROOT / "index.html").read_text(encoding="utf-8")

async def main():
    async with async_playwright() as p:
        browser = await p.chromium.launch(headless=True)
        page = await browser.new_page(viewport={"width": 1440, "height": 1000})
        mock = """<script>(()=>{const tools={};const mc={async registerTool(tool){tools[tool.name]=tool;}};Object.defineProperty(Document.prototype,'modelContext',{configurable:true,get(){return mc;}});window.__webmcpTools=tools;})();</script>"""
        await page.set_content(HTML.replace("<head>", "<head>"+mock, 1), wait_until="load")
        await page.wait_for_function("document.querySelector('#webmcpBadge').textContent.includes('registered')")
        assert await page.locator("#topicNav button").count() == 28
        await page.evaluate("window.__webmcpTools.cartpole_set_topic.execute({topic:'full_nmpc'})")
        await page.click("#useTopic")
        state = json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_steps.execute({steps:8})"))
        assert state["controller"] == "full_nmpc"
        assert state["solver"]["horizon"] == 30
        await page.evaluate("window.__webmcpTools.cartpole_set_topic.execute({topic:'centroidal_mpc'})")
        await page.click("#useTopic")
        state2 = json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_steps.execute({steps:8})"))
        assert state2["controller"] == "centroidal_mpc"
        assert state2["solver"]["horizon"] == 32

        await page.evaluate("window.__webmcpTools.cartpole_set_controller.execute({controller:'state_mpc'})")
        state_mpc_state = json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_steps.execute({steps:4})"))
        assert state_mpc_state["controller"] == "state_mpc"

        await page.evaluate("window.__webmcpTools.cartpole_set_controller.execute({controller:'ltv_mpc'})")
        ltv_state = json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_steps.execute({steps:4})"))
        assert ltv_state["controller"] == "ltv_mpc"
        assert ltv_state["solver"]["horizon"] == 30
        assert ltv_state["solver"]["iterations"] == 1

        await page.evaluate("window.__webmcpTools.cartpole_set_observer.execute({observer:'ukf'})")
        ukf_state = json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_steps.execute({steps:8})"))
        assert ukf_state["observer"] == "ukf"
        assert ukf_state["P"] is not None

        await page.evaluate("window.__webmcpTools.cartpole_set_observer.execute({observer:'mhe'})")
        mhe_state = json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_steps.execute({steps:12})"))
        assert mhe_state["observer"] == "mhe"
        assert mhe_state["observerDiagnostics"]["iterations"] > 0
        assert mhe_state["observerDiagnostics"]["cost"] >= 0

        await page.evaluate("window.__webmcpTools.cartpole_set_scenario.execute({scenario:'sim2real'})")
        s2r_state = json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_steps.execute({steps:4})"))
        assert s2r_state["scenario"] == "sim2real"

        # Nonlinear teaching preset must be reachable through the semantic surface.
        await page.evaluate("window.__webmcpTools.cartpole_set_scenario.execute({scenario:'nonlinear'})")
        nonlinear_state = json.loads(await page.evaluate("window.__webmcpTools.cartpole_get_state.execute({})"))
        assert nonlinear_state["scenario"] == "nonlinear"
        assert abs(nonlinear_state["truth"][2]) > 0.4

        kf_probe = json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_probe.execute({controller:'lqr',observer:'kf',scenario:'nonlinear',seed:4,steps:250,pushForce:0})"))
        ekf_probe = json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_probe.execute({controller:'lqr',observer:'ekf',scenario:'nonlinear',seed:4,steps:250,pushForce:0})"))
        assert ekf_probe["rmseState"] < 0.6 * kf_probe["rmseState"]

        actuator_probe = json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_probe.execute({controller:'lqr',observer:'ekf',scenario:'actuator',seed:4,steps:120,pushForce:0})"))
        assert actuator_probe["rmsCommandMismatch"] > 0
        dropout_probe = json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_probe.execute({controller:'lqr',observer:'ekf',scenario:'dropout',seed:4,steps:120,pushForce:0})"))
        assert any(q.get("sensorFresh") is False for q in dropout_probe["trace"])
        assert all(q.get("measurementUsed") is False and q["S"] is None and q["innovation"] is None for q in dropout_probe["trace"] if not q["sensorFresh"])
        thermal_probe = json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_probe.execute({controller:'lqr',observer:'ekf',scenario:'thermal',seed:4,steps:250,pushForce:0})"))
        assert max(q.get("thermalState",0) for q in thermal_probe["trace"]) > 0

        await page.evaluate("window.__webmcpTools.cartpole_set_scenario.execute({scenario:'actuator'})")
        await page.evaluate("window.__webmcpTools.cartpole_run_steps.execute({steps:12})")
        live = json.loads(await page.evaluate("window.__webmcpTools.cartpole_get_state.execute({})"))
        assert "appliedForce" in live

        await page.evaluate("window.__webmcpTools.cartpole_set_observer.execute({observer:'ekf'})")
        await page.evaluate("window.__webmcpTools.cartpole_set_scenario.execute({scenario:'dropout'})")
        missing = json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_steps.execute({steps:3})"))
        assert missing["sensorFresh"] is False and missing["measurementUsed"] is False
        assert missing["innovation"] is None
        assert await page.locator("#mInnov").text_content() == "—"
        assert missing["measurementCovarianceSource"] == "injected-noise-oracle"

        assert await page.evaluate("document.documentElement.scrollWidth-window.innerWidth") <= 1
        await browser.close()
        print("browser smoke: PASS")

asyncio.run(main())
