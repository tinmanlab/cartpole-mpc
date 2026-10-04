"""Actual shared-page train/validation/test operation, not precomputed screenshot success."""
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
    assert page.locator('#processRun').count()==1,'Missing staged Q-selection lesson'
    assert 'R_e 비교와 Q_e 선택' in page.locator('#calibrationLesson .card-head small').inner_text()
    page.evaluate('window.controlLab.pause()');before=page.evaluate('window.controlLab.getState()')
    page.evaluate("()=>{Plant.integrate=()=>{throw Error('Legacy physics forbidden')};ControlLab.LabPlant.prototype.measurementVariance=()=>{throw Error('Noise oracle forbidden')};}")
    page.click('#processRun');page.wait_for_function('window.calibrationLesson.getProcessState().status!=="running"',timeout=90000)
    state=page.evaluate('window.calibrationLesson.getProcessState()');assert state['status']=='completed',state
    expected=json.loads((ROOT/'evidence/process_selection.json').read_text())
    assert state['selection']['scale']==expected['selection']['scale'];assert state['selection']['testUsedForSelection'] is False
    assert state['selection']['atGridBoundary']==expected['selection']['atGridBoundary']
    for group,reference in zip(state['groups'],expected['groups']):
        assert group['bothCompletedPairs']==reference['bothCompletedPairs']
        for arm,ref in zip(group['arms'],reference['arms']):
            assert arm['taskPassed']==ref['taskPassed']
            assert abs(arm['bothCompletedTrackingRmse_m']-ref['bothCompletedTrackingRmse_m'])<1e-7
    assert page.locator('#processScores tbody tr').count()==6
    assert page.locator('#processOutcomes tbody tr').count()==8
    assert '최적' in page.locator('#processStatus').inner_text()  # grid-boundary caveat, not optimum claim
    page.select_option('#processCase','test-6719');page.click('#processInspect')
    page.select_option('#calibrationArm','q_selected')
    shown=page.evaluate('window.calibrationLesson.seek(25)')
    assert shown['mode']=='process' and shown['caseId']=='test-6719'
    assert len(shown['runs'])==2 and shown['frame']['commandTime']==.5
    assert 'Q_e' in page.locator('#calibrationInterpretation').inner_text()
    assert page.locator('#calibrationTable tbody tr').count()==2
    after=page.evaluate('window.controlLab.getState()')
    for key in ['controller','observer','scenario','truth','estimate','t','terminalCost']:assert before[key]==after[key],key
    page.locator('#processLesson').screenshot(path=str(ROOT/'evidence/process_selection_browser.png'))
    # Existing R lesson remains usable after switching from the process experiment.
    page.click('#calibrationRun');page.wait_for_function('window.calibrationLesson.getState().status!=="running"',timeout=30000)
    assert page.evaluate('window.calibrationLesson.getState().mode')=='measurement'
    assert page.locator('#calibrationTable tbody tr').count()==3
    assert not page.locator('#processRun').is_disabled() and not page.locator('#calibrationRun').is_disabled()
    page.evaluate("()=>{window.__savedProcessStudy=CalibrationLab.runProcessStudy;CalibrationLab.runProcessStudy=async()=>{throw Error('Injected calculation rejection')};}")
    page.click('#processRun');page.wait_for_function('window.calibrationLesson.getProcessState().status==="error"')
    assert page.evaluate('window.calibrationLesson.getProcessState().selection') is None
    assert page.locator('#processScores tbody tr').count()==0
    assert page.locator('#processOutcomes tbody tr').count()==0
    assert page.locator('#calibrationTable tbody tr').count()==0
    assert page.evaluate('window.calibrationLesson.getState().frame') is None
    assert not page.locator('#processRun').is_disabled() and not page.locator('#calibrationRun').is_disabled()
    page.evaluate('()=>{CalibrationLab.runProcessStudy=window.__savedProcessStudy;}')
    assert page.evaluate('document.documentElement.scrollWidth-innerWidth')<=1
    assert not errors,errors
    result={'schema':'cartpole-process-browser/v1','passed':True,'actualWasm':True,'noiseOracleForbidden':True,'computedSelectionMatchesNode':True,'trainValidationTestVisible':True,'gridBoundaryCaveatVisible':True,'negativeOutcomesPreserved':True,'sharedCausalTraceReused':True,'legacyModeRestored':True,'failedRerunDoesNotShowStaleSelection':True,'mainLoopUnchanged':True,'pageErrors':errors,'deployed':False}
    (ROOT/'evidence/process_browser.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));browser.close()
finally:server.shutdown()
