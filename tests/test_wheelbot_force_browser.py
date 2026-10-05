"""Actual pulse UI and numerical parity; only run in normal authorized CI."""
from pathlib import Path
import functools,http.server,json,threading
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
    page.goto(f'http://127.0.0.1:{server.server_port}/wheelbot.html',wait_until='networkidle')
    page.wait_for_function('window.wheelbotLab?.ready',timeout=60000)
    report=json.loads((ROOT/'evidence/wheelbot_force_envelope.json').read_text())
    assert not page.locator('#force-left').is_disabled() and not page.locator('#force-right').is_disabled()
    assert str(report['normal']['amplitudeN']) in page.locator('#force-level').inner_text()
    for mode in ('lqr_kf','mpc_kf'):
        page.evaluate('(mode)=>window.wheelbotLab.configureMode(mode)',mode)
        for seed in (901,907,911,919,929):
            for direction in (-1,1):
                start=page.evaluate('([direction,seed])=>window.wheelbotLab.prepareForce(direction,"normal",seed)',[direction,seed])
                assert start['forceActive'] and not start['jumpActive'] and not start['playing'] and start['steps']==0
                end=page.evaluate('window.wheelbotLab.run(300)')
                assert end['steps']==300 and end['done'] and end['forceResult']['normalPassed']
                expected=next(r for r in report['assessment'] if r['mode']==mode and r['seed']==seed and r['direction']==direction)
                assert max(abs(a-b)for a,b in zip(end['truth'],expected['final']))<1e-7
                assert abs(end['forceResult']['appliedSignedImpulseNs']-direction*.2*report['normal']['amplitudeN'])<1e-12
                count=end['physics']['steps'];assert page.evaluate('window.wheelbotLab.run(20).physics.steps')==count
    page.screenshot(path=str(ROOT/'evidence/wheelbot_force_normal.png'),full_page=True)
    assert page.locator('#force-level option').count()==0
    assert '40 N' not in page.locator('body').inner_text()
    # Public rejection must preserve the whole simulation state, including steps.
    rejected=page.evaluate("""() => {
      const lab=window.wheelbotLab;
      return ['stress','40',40,NaN,null,''].map(level=>{
        const before=JSON.stringify(lab.getState()); let error='';
        try { lab.prepareForce(1,level); } catch(e) { error=e.message; }
        return {error,unchanged:before===JSON.stringify(lab.getState())};
      });
    }""")
    assert all('normal' in row['error'] and row['unchanged'] for row in rejected)
    page.click('#reset');reset=page.evaluate('window.wheelbotLab.getState()')
    assert not reset['forceActive'] and reset['steps']==0 and not page.locator('#mode').is_disabled()
    # Actual button path includes bounded scheduler execution of all300steps.
    page.click('#force-left');page.wait_for_function('window.wheelbotLab.getState().done',timeout=30000)
    assert page.evaluate('window.wheelbotLab.getState().forceResult.normalPassed')
    # Stale receipt only disables the pulse task, not standing control or jump.
    bad=json.loads(json.dumps(report));bad['profileSha256']='0'*64
    def stale(route):route.fulfill(status=200,content_type='application/json',body=json.dumps(bad))
    page.route('**/evidence/wheelbot_force_envelope.json',stale)
    page.reload(wait_until='networkidle');page.wait_for_function('window.wheelbotLab?.ready',timeout=60000)
    assert page.locator('#force-left').is_disabled() and page.locator('#force-right').is_disabled()
    assert 'unavailable' in page.locator('#force-status').inner_text()
    standing=page.evaluate('window.wheelbotLab.run(20)');assert standing['steps']==20 and not standing['failed']
    assert not errors,errors
    out={'schema':'wheelbot-force-browser/v1','passed':True,'actualWasm':True,'bothDirectionsControllersMatchNumericAssessment':True,'durationAndImpulseVerified':True,'unsupportedForceRejectedWithoutStateChange':True,'expiredTrialDoesNotAdvance':True,'resetRestoresStanding':True,'actualButtonRuns300Steps':True,'staleReceiptPreservesStanding':True,'pageErrors':errors,'deployed':False}
    (ROOT/'evidence/wheelbot_force_browser.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out));browser.close()
finally:server.shutdown()
