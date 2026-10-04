"""Real shared-page selection, independent-test verdict and explicit live application."""
import functools,http.server,threading,json
from pathlib import Path
from playwright.sync_api import sync_playwright
ROOT=Path(__file__).resolve().parents[1]
class Quiet(http.server.SimpleHTTPRequestHandler):
    def log_message(self,*args):pass
server=http.server.ThreadingHTTPServer(('127.0.0.1',0),functools.partial(Quiet,directory=str(ROOT)));threading.Thread(target=server.serve_forever,daemon=True).start()
try:
 with sync_playwright() as p:
    browser=p.chromium.launch(headless=True);page=browser.new_page(viewport={'width':1440,'height':1100});errors=[]
    page.on('pageerror',lambda e:errors.append(str(e)))
    page.goto(f'http://127.0.0.1:{server.server_port}/index.html',wait_until='networkidle');page.wait_for_function('window.__labReady || window.__labLoadError')
    assert page.evaluate('window.__labLoadError || null') is None
    assert page.locator('#designRun').count()==1,'Missing common-task four-pair learning path'
    page.evaluate('window.controlLab.pause()');before=page.evaluate('window.controlLab.getState()')
    page.evaluate("()=>{Plant.integrate=()=>{throw Error('Legacy physics forbidden')};window.__savedVariance=ControlLab.LabPlant.prototype.measurementVariance;ControlLab.LabPlant.prototype.measurementVariance=()=>{throw Error('Noise oracle forbidden')};}")
    page.click('#designRun');page.wait_for_function('window.designLesson.getState().status!=="running"',timeout=180000)
    study=page.evaluate('window.designLesson.getState()');assert study['status']=='completed',study
    ref=json.loads((ROOT/'evidence/design_study.json').read_text())
    assert study['selection']['recommendedPairId']==ref['selection']['recommendedPairId']
    assert study['admission']['accepted']==ref['admission']['accepted']
    assert [(r['pairId'],r['candidate']['id']) for r in study['selection']['pairs']]==[(r['pairId'],r['candidate']['id']) for r in ref['selection']['pairs']]
    assert page.locator('#designPairs tbody tr').count()==4
    assert page.locator('#designCandidates tbody tr').count()==9
    assert page.locator('#designTests tbody tr').count()==6
    assert '전역' in page.locator('#designVerdict').inner_text()
    after=page.evaluate('window.controlLab.getState()')
    for key in ['controller','observer','truth','estimate','t','terminalCost']:assert before[key]==after[key],key
    applied=False
    if study['admission']['accepted']:
        assert not page.locator('#designApply').is_disabled()
        cid='test-mixed-7433';page.select_option('#designApplyCase',cid);page.click('#designApply')
        state=page.evaluate('window.controlLab.getState()')
        assert not state['running'] and state['study']['caseId']==cid and state['t']==0
        assert state['measurementCovarianceSource']=='stationary-measurements'
        expected_asset=page.evaluate("()=>{const p=new ControlLab.LabPlant({scenario:'mixed'});return 'true rod '+(2*p.params.l).toFixed(3)+' m';}")
        assert expected_asset in page.locator('#assetStatus').inner_text(), 'Applied model differs from displayed asset dimensions'
        assert state['study']['design']['candidate']['id']==next(q['candidate']['id'] for q in ref['selection']['pairs'] if q['pairId']==ref['selection']['recommendedPairId'])
        page.evaluate('window.controlLab.step(300)');state=page.evaluate('window.controlLab.step(300)')
        expected=next(r['candidate'] for r in ref['tests'] if r['pairId']==ref['selection']['recommendedPairId'] and r['caseId']==cid)
        assert state['study']['run']['taskPassed']==expected['taskPassed']
        assert abs(state['study']['run']['fullScore']-expected['fullScore'])<1e-7
        assert max(abs(a-b) for a,b in zip(state['truth'],expected['finalTruth']))<1e-7
        assert state['t']==12 and not state['running']
        # An ordinary manual setting change explicitly exits the admitted task recipe.
        page.evaluate('()=>{ControlLab.LabPlant.prototype.measurementVariance=window.__savedVariance;}')
        page.select_option('#controller','hard_mpc');assert page.evaluate('window.controlLab.getState().study') is None
        applied=True
    else:assert page.locator('#designApply').is_disabled()
    page.locator('#designLesson').screenshot(path=str(ROOT/'evidence/design_study_browser.png'))
    page.evaluate("()=>{window.__savedStudy=DesignStudy.runStudy;DesignStudy.runStudy=async()=>{throw Error('Injected study failure')};}")
    page.click('#designRun');page.wait_for_function('window.designLesson.getState().status==="error"')
    assert page.evaluate('window.designLesson.getState().selection') is None
    assert page.locator('#designApply').is_disabled() and page.locator('#designPairs tbody tr').count()==0
    assert not page.locator('#designRun').is_disabled()
    assert not errors,errors
    assert page.evaluate('document.documentElement.scrollWidth-innerWidth')<=1
    out={'schema':'cartpole-design-browser/v1','passed':True,'actualWasm':True,'noNoiseOracle':True,'computedSelectionMatchesNode':True,'noAutomaticStateOrDefaultChange':True,'liveAppliedAndReplayedMatchedCase':applied,'ordinaryConfigExitsRecipe':applied,'appliedAssetReadoutMatchesModel':applied,'failedRerunClearsRecommendation':True,'pageErrors':errors,'webmcpShim':False,'deployed':False}
    (ROOT/'evidence/design_browser.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out));browser.close()
finally:server.shutdown()
