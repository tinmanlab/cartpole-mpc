"""Actual HTTP/WASM jump, landing, reset and optional-profile rejection.
This runs in normal CI; do not retry it in a socket-denied sandbox.
"""
from pathlib import Path
import hashlib
import functools
import http.server
import json
import threading
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[1]
class Quiet(http.server.SimpleHTTPRequestHandler):
    def log_message(self,*args):pass
server=http.server.ThreadingHTTPServer(('127.0.0.1',0),functools.partial(Quiet,directory=str(ROOT)))
threading.Thread(target=server.serve_forever,daemon=True).start()
try:
    with sync_playwright() as p:
        browser=p.chromium.launch(headless=True)
        page=browser.new_page(viewport={'width':1440,'height':1100})
        errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
        url=f'http://127.0.0.1:{server.server_port}/wheelbot.html'
        page.goto(url,wait_until='networkidle')
        page.wait_for_function('window.wheelbotLab?.ready',timeout=60000)
        assert not page.locator('#jump').is_disabled()
        before=page.evaluate('window.wheelbotLab.getState()')
        initial=page.evaluate('window.wheelbotLab.prepareJump(809)')
        assert initial['jumpActive'] and initial['steps']==0 and not initial['playing']
        assert initial['truth']==before['truth']
        assert page.locator('#mode').is_disabled() and page.locator('#design').is_disabled()
        airborne=page.evaluate('window.wheelbotLab.run(100)')
        assert airborne['steps']==100 and not airborne['failed']
        assert airborne['physics']['steps']-initial['physics']['steps']==500
        assert airborne['last']['geometry']['wheelContacts']==0
        assert airborne['last']['geometry']['wheelClearanceM']>.005
        assert 'FLIGHT' in page.locator('#jump-status').inner_text()
        page.screenshot(path=str(ROOT/'evidence/wheelbot_jump_airborne.png'),full_page=True)
        landed=page.evaluate('window.wheelbotLab.run(300)')
        assert landed['steps']==400 and landed['done'] and landed['jumpResult']['passed'],landed['jumpResult']
        assert landed['jumpResult']['externalForceIsZero']
        assert landed['physics']['steps']-initial['physics']['steps']==2000
        expected=json.loads((ROOT/'evidence/wheelbot_jump_reference.json').read_text())
        assert expected['profileSha256']==hashlib.sha256((ROOT/'assets/wheelbot/jump_profile.json').read_bytes()).hexdigest()
        ref=next(r for r in expected['rows'] if r['mode']=='tvlqr_kf' and r['seed']==809)
        assert abs(ref['comApexM']-landed['jumpResult']['comApexM'])<1e-7
        assert abs(ref['tailPositionM']-landed['jumpResult']['tailPositionM'])<1e-7
        count=landed['physics']['steps']
        assert page.evaluate('window.wheelbotLab.run(10).physics.steps')==count
        assert 'target met' in page.locator('#jump-status').inner_text()
        page.screenshot(path=str(ROOT/'evidence/wheelbot_jump_landed.png'),full_page=True)
        page.click('#reset')
        reset=page.evaluate('window.wheelbotLab.getState()')
        assert reset['steps']==0 and not reset['jumpActive']
        assert not page.locator('#mode').is_disabled()
        # Actual click/play loop, not only the testing API.
        page.click('#jump')
        page.wait_for_function('window.wheelbotLab.getState().done',timeout=30000)
        assert page.evaluate('window.wheelbotLab.getState().jumpResult.passed')
        assert page.evaluate('window.wheelbotLab.getState().steps')==400
        assert not page.evaluate('window.wheelbotLab.getState().playing')
        # Mismatched optional jump must not break the standing lab.
        profile=json.loads((ROOT/'assets/wheelbot/jump_profile.json').read_text())
        profile['baselineSha256']='0'*64
        def wrong_profile(route):
            route.fulfill(status=200,content_type='application/json',body=json.dumps(profile))
        page.route('**/assets/wheelbot/jump_profile.json',wrong_profile)
        page.reload(wait_until='networkidle');page.wait_for_function('window.wheelbotLab?.ready',timeout=60000)
        assert page.locator('#jump').is_disabled()
        assert 'Jump unavailable' in page.locator('#jump-status').inner_text()
        fallback=page.evaluate('window.wheelbotLab.run(20)')
        assert fallback['steps']==20 and not fallback['failed'] and not fallback['jumpActive']
        assert not errors,errors
        assert page.evaluate('document.documentElement.scrollWidth-innerWidth')<=1
        result={'schema':'wheelbot-jump-browser/v1','passed':True,'actualWasm':True,'airborneAt100':True,'landingAndSettlingPassed':True,'noPostCompletionAdvance':True,'actualButtonRuns400Steps':True,'explicitResetRestoresStanding':True,'mismatchedProfilePreservesStanding':True,'pageErrors':errors,'deployed':False}
        (ROOT/'evidence/wheelbot_jump_browser.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result));browser.close()
finally:server.shutdown()
