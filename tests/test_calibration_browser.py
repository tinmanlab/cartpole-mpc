"""Same published-page code, real WASM, actual lesson controls; no WebMCP shim."""
from pathlib import Path
import functools,http.server,threading,json
from playwright.sync_api import sync_playwright
ROOT=Path(__file__).resolve().parents[1]
class Quiet(http.server.SimpleHTTPRequestHandler):
    def log_message(self,*args):pass
server=http.server.ThreadingHTTPServer(('127.0.0.1',0),functools.partial(Quiet,directory=str(ROOT)))
threading.Thread(target=server.serve_forever,daemon=True).start()
try:
 with sync_playwright() as p:
    browser=p.chromium.launch(headless=True);page=browser.new_page(viewport={'width':1440,'height':1100});errors=[]
    page.on('pageerror',lambda e:errors.append(str(e)))
    page.goto(f'http://127.0.0.1:{server.server_port}/index.html',wait_until='networkidle')
    page.wait_for_function('window.__labReady || window.__labLoadError');assert page.evaluate('window.__labLoadError || null') is None
    assert page.locator('#calibrationLesson').count()==1,'Missing causal calibration panel'
    page.evaluate('window.controlLab.pause()');before=page.evaluate('window.controlLab.getState()')
    page.evaluate("()=>{Plant.integrate=()=>{throw Error('Legacy physics is not allowed')};ControlLab.LabPlant.prototype.measurementVariance=()=>{throw Error('noise oracle used')};}")
    page.click('#calibrationRun');page.wait_for_function('window.calibrationLesson.getState().status!=="running"',timeout=30000)
    state=page.evaluate('window.calibrationLesson.getState()');assert state['status']=='completed',(state,page.locator('#calibrationStatus').inner_text())
    assert state['caseId']=='white-6301' and len(state['runs'])==3
    ref=json.loads((ROOT/'evidence/calibration_lesson.json').read_text());expected=ref['cases'][0]['runs']
    for actual,e in zip(state['runs'],expected):
        assert actual['outcome']==e['outcome'] and actual['appliedSteps']==e['appliedSteps']
        assert abs(actual['positionTrackingRmse_m']-e['positionTrackingRmse_m'])<1e-7
        assert actual['saturatedSamples']==e['saturatedSamples']
    after=page.evaluate('window.controlLab.getState()')
    for key in ['controller','observer','scenario','truth','estimate','t','terminalCost']:assert before[key]==after[key],key
    assert page.locator('#calibrationTable tbody tr').count()==3
    page.select_option('#calibrationArm','nominal_assumption')
    snapshot=page.evaluate('window.calibrationLesson.seekFirstSaturation()');assert snapshot['frame']['saturated']
    assert abs(snapshot['frame']['requestedForce']-snapshot['frame']['oracleForce']-sum(snapshot['frame']['estimationForceContributions']))<1e-9
    assert '평가용' in page.locator('#calibrationForce').inner_text()
    assert 'K_e 일부' in page.locator('#calibrationEstimate').inner_text()
    assert snapshot['frame']['filterGain'] is not None
    assert 'm²' in page.locator('#calibrationParameters').inner_text()
    assert page.locator('#calibrationTimeline').get_attribute('data-units')=='m,deg,N'
    page.locator('#calibrationInspect details summary').click()
    assert '동일한 측정·입력' in page.locator('#calibrationReplay').inner_text()
    assert not page.locator('#calibrationRun').is_disabled()
    page.locator('#calibrationLesson').screenshot(path=str(ROOT/'evidence/calibration_lesson_browser.png'))
    rejected=page.evaluate("async()=>{try{await window.calibrationLesson.run('invalid-case');return false;}catch(e){return true;}}")
    assert rejected and not page.locator('#calibrationRun').is_disabled()
    assert page.evaluate('document.documentElement.scrollWidth-innerWidth')<=1
    assert not errors,errors
    out={'schema':'cartpole-calibration-browser/v1','passed':True,'actualWasm':True,'noiseOracleForbidden':True,'legacyMainStateAndSettingsUnchanged':True,'sameSourceNumericalMatch':True,'saturationCausalFrameVerified':True,'sourceScopeUnitsReplayVerified':True,'pageErrors':errors,'deployed':False}
    (ROOT/'evidence/calibration_browser.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out));browser.close()
finally:server.shutdown()
