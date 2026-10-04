from pathlib import Path
import asyncio, json, functools, http.server, threading
from playwright.async_api import async_playwright

ROOT = Path(__file__).resolve().parents[1]
class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self,*args): pass
SERVER=http.server.ThreadingHTTPServer(('127.0.0.1',0),functools.partial(QuietHandler,directory=str(ROOT)))
threading.Thread(target=SERVER.serve_forever,daemon=True).start()
URL=f'http://127.0.0.1:{SERVER.server_port}/'


async def main():
    async with async_playwright() as p:
        browser = await p.chromium.launch(headless=True)
        page = await browser.new_page(viewport={"width": 1440, "height": 1000})
        mock = """<script>(()=>{const tools={};const mc={async registerTool(tool){tools[tool.name]=tool;}};Object.defineProperty(Document.prototype,'modelContext',{configurable:true,get(){return mc;}});window.__webmcpTools=tools;})();</script>"""
        errors=[];responses=[]
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.on('response',lambda r:responses.append({'url':r.url,'status':r.status}))
        await page.add_init_script(mock.removeprefix('<script>').removesuffix('</script>'))
        await page.goto(URL,wait_until='networkidle')
        await page.wait_for_function('window.__labReady || window.__labLoadError',timeout=60000)
        assert await page.evaluate('window.__labLoadError || null') is None
        assert await page.locator('#terminalCost').count()==1, 'Missing terminal-cost control'
        await page.evaluate('window.controlLab.pause()')
        # A fallback to the old equation port is now an explicit test failure.
        await page.evaluate("() => {Plant.integrate=()=>{throw Error('legacy JS plant was called')}; return true;}")

        await page.wait_for_function("document.querySelector('#webmcpBadge').textContent.includes('registered')")
        assert await page.locator("#topicNav button").count() == 28
        await page.evaluate("window.__webmcpTools.cartpole_set_topic.execute({topic:'full_nmpc'})")
        await page.click("#useTopic")
        state = json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_steps.execute({steps:8})"))
        assert state["controller"] == "full_nmpc"
        assert state["physics"]["backend"] == "mujoco-wasm"
        assert state["physics"]["version"] == "3.7.0"
        assert state["physics"]["stepCalls"] > 0
        assert state["fault"] is None
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
        assert not errors, errors
        assert any(r['url'].endswith('mujoco.wasm') and r['status']==200 for r in responses)
        assert any(r['url'].endswith('cartpole.xml') and r['status']==200 for r in responses)
        # Second page has NO WebMCP shim: normal UI, actual network assets and physics.
        native_page=await browser.new_page(viewport={'width':1440,'height':1000})
        native_errors=[];native_page.on('pageerror',lambda e:native_errors.append(str(e)))
        await native_page.goto(URL,wait_until='networkidle')
        await native_page.wait_for_function('window.__labReady || window.__labLoadError')
        assert await native_page.evaluate('window.__labLoadError || null') is None
        await native_page.select_option('#controller','linear_mpc')
        await native_page.select_option('#observer','ekf')
        await native_page.evaluate('window.controlLab.pause()')
        final=await native_page.evaluate('window.controlLab.step(20)')
        assert final['solver']['implementation']=='quadprog-goldfarb-idnani'
        assert final['solver']['kktResidual']<1e-7
        assert final['physics']['backend']=='mujoco-wasm'
        # The browser-packaged solver, not just Node require(), must match OSQP.
        refs=json.loads((ROOT/'evidence/native_reference.json').read_text())['qp_cases']
        browser_actions=await native_page.evaluate('''cases=>cases.map(r=>{const c=new ControlLab.LinearMPCController(new ControlLab.LabPlant().spec);const u=c.act(r.x0,0);return {u,residual:c.lastKktResidual};})''',refs)
        for a,r in zip(browser_actions,refs):
            assert abs(a['u']-r['native']['action'])<0.001
            assert a['residual']<1e-7
        await native_page.select_option('#controller','hard_mpc')
        await native_page.select_option('#goal','0.5')
        await native_page.evaluate('window.controlLab.pause()')
        hard=await native_page.evaluate('window.controlLab.step(60)')
        assert hard['fault'] is None and hard['solver']['stateConstrained']
        assert hard['solver']['primalResidual']<1e-8
        assert abs(hard['truth'][0])<2.4
        assert hard['timing']['postStateTime']>hard['timing']['sourceStateTime']
        assert hard['timing']['computeMs']>=hard['timing']['solveToApplyMs']>=0
        await native_page.screenshot(path=str(ROOT/'evidence/mujoco_wasm_browser.png'),full_page=True)
        # Actual terminal selector must drive the public constructor, not a test-only matrix assignment.
        await native_page.select_option('#terminalCost','original')
        await native_page.evaluate('window.controlLab.pause()')
        old=await native_page.evaluate('window.controlLab.step(1)')
        assert old['terminalCost']['kind']=='original'
        await native_page.select_option('#terminalCost','dare')
        await native_page.evaluate('window.controlLab.pause()')
        ric=await native_page.evaluate('window.controlLab.step(80)')
        assert ric['fault'] is None and ric['terminalCost']['kind']=='dare'
        assert ric['terminalCost']['source']=='nominal-discrete-riccati'
        assert ric['terminalCost']['normalizedResidual']<=1e-9
        assert abs(ric['terminalCost']['matrix'][0][1])>1
        assert await native_page.evaluate("window.controlLab.getTrace().every(q=>q.terminalCost==='dare')")
        await native_page.click('#reset')
        assert await native_page.evaluate("window.controlLab.getState().terminalCost.kind")=='dare'
        await native_page.select_option('#controller','lqr')
        assert await native_page.locator('#terminalCost').is_disabled()
        assert await native_page.evaluate('window.controlLab.getState().terminalCost') is None
        await native_page.select_option('#controller','hard_mpc')
        assert not await native_page.locator('#terminalCost').is_disabled()
        assert await native_page.evaluate("window.controlLab.getState().terminalCost.kind")=='dare'
        await native_page.evaluate('window.controlLab.pause()')
        await native_page.screenshot(path=str(ROOT/'evidence/terminal_option_browser.png'),full_page=True)
        await page.evaluate("window.__webmcpTools.cartpole_set_controller.execute({controller:'hard_mpc',terminalCost:'dare'})")
        probe=json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_probe.execute({controller:'hard_mpc',observer:'ekf',scenario:'nominal',steps:60,pushForce:0})"))
        assert probe['terminalCost']['kind']=='dare'
        assert all(q['terminalCost']=='dare' for q in probe['trace'])
        explicit=json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_probe.execute({controller:'hard_mpc',observer:'ekf',scenario:'nominal',terminalCost:'original',steps:60,pushForce:0})"))
        assert explicit['terminalCost']['kind']=='original'
        rejected=await page.evaluate("async()=>{try{await window.__webmcpTools.cartpole_set_controller.execute({controller:'pid',terminalCost:'dare'});return false;}catch(e){return /unsupported/i.test(e.message);}}")
        assert rejected
        # Fault injection checks only the comparison UI boundary; all physics tests above are real.
        await native_page.evaluate("() => {window.__comparedTerminal=[];window.__savedEpisode=ControlLab.runEpisode;ControlLab.runEpisode=o=>{window.__comparedTerminal.push({controller:o.controller,terminal:o.controllerOpts?.terminalCost??null});if(o.controller==='hard_mpc')throw Error('QP rejected: injected infeasible');return {failed:false,rmseState:0,meanSolveMs:0};};}")
        await native_page.click('#compare')
        await native_page.wait_for_function("document.querySelector('#matrix').textContent.includes('REJECT') && !document.querySelector('#compare').disabled",timeout=1500)
        assert await native_page.locator('#matrix tbody tr').count()==11
        assert await native_page.evaluate("window.__comparedTerminal.every(q=>['linear_mpc','hard_mpc'].includes(q.controller)?q.terminal==='dare':q.terminal===null)")
        await native_page.evaluate("() => {ControlLab.runEpisode=window.__savedEpisode;delete window.__savedEpisode;}")
        # Invalid output must never advance the plant or silently apply zero.
        fault=await native_page.evaluate('''()=>{window.controlLab.pause();const t=window.controlLab.getState().t;
            const original=ControlLab.LinearMPCController.prototype.act;ControlLab.LinearMPCController.prototype.act=()=>NaN;
            const state=window.controlLab.step(1);ControlLab.LinearMPCController.prototype.act=original;return {before:t,state};}''')
        assert fault['state']['t']==fault['before'] and fault['state']['fault']
        assert fault['state']['running'] is False
        await native_page.click('#reset')
        assert await native_page.evaluate('window.controlLab.getState().fault') is None
        # A rejected constructor must not leave a new plant running with an old controller.
        init_fault=await native_page.evaluate("""()=>{
            window.controlLab.pause();const before=window.controlLab.getState().t;
            const original=ControlLab.makeController;
            ControlLab.makeController=()=>{throw Error('Riccati terminal computation did not converge')};
            document.querySelector('#terminalCost').dispatchEvent(new Event('change'));
            ControlLab.makeController=original;
            const stopped=window.controlLab.step(2);return {before,stopped};
        }""")
        assert init_fault['stopped']['fault'] and not init_fault['stopped']['running']
        assert init_fault['stopped']['t']==init_fault['before']
        await native_page.click('#reset')
        assert await native_page.evaluate('window.controlLab.getState().fault') is None
        # Missing binary must visibly fail closed; the legacy JS path is forbidden.
        blocked=await browser.new_page()
        await blocked.route('**/mujoco.wasm',lambda route:route.abort())
        await blocked.goto(URL,wait_until='networkidle')
        await blocked.wait_for_function('window.__labLoadError',timeout=60000)
        assert await blocked.locator('#controller').is_disabled()
        assert await blocked.evaluate('window.__labReady || false') is False
        await blocked.close()

        assert not native_errors,native_errors
        result={'schema':'cartpole-browser-wasm/v1','physics':final['physics'],'solver':final['solver'],
            'wasmHttp200':True,'mjcfHttp200':True,'legacyIntegratorForbidden':True,'pageErrors':errors+native_errors,
            'webmcpNativeAvailable':await native_page.evaluate("!!((document.modelContext||navigator.modelContext)?.registerTool)"),
            'webmcpAdapterTest':'first page only uses an explicitly injected registration shim',
            'normalUiWithoutShimPassed':True,'hardRailControllerPassed':True,'browserQpParityPassed':True,'invalidCommandStoppedBeforePlant':True,'missingWasmFailClosed':True,'terminalOptionActualBrowserPassed':True,'terminalConstructorFailureStopped':True,'terminalProbeAndComparisonPassed':True,'comparisonRejectionIsolated':True,'timing':hard['timing'],'hardwareVerified':False}
        (ROOT/'evidence/browser_wasm.json').write_text(json.dumps(result,indent=2)+'\n')
        await browser.close()
        SERVER.shutdown()

        print("browser smoke: PASS")

asyncio.run(main())
