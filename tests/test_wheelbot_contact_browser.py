"""Real configurable model, falling contacts, profile invalidation and restoration.
Run in the normal CI browser; no socket workaround in developer sandboxes.
"""
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
    page.wait_for_function('window.wheelbotLab?.ready && window.wheelbotLab.getDesignEditor()',timeout=60000)
    original=page.evaluate('window.wheelbotLab.getState()')
    assert not original['configuredModel']
    page.locator('#physical-editor').evaluate('(e)=>e.open=true')
    page.click('#physical-apply');page.wait_for_function('window.wheelbotLab.getState().configuredModel',timeout=60000)
    page.wait_for_function('!window.wheelbotLab.getDesignEditor().busy')
    assert 'FULL BODY CONTACT' in page.locator('#physical-status').inner_text()
    info=page.evaluate('window.wheelbotLab.getDesignEditor()');assert info['profileValid']
    assert page.evaluate('window.wheelbotLab.selectDesign("response").accepted') is False
    assert page.locator('#jump').is_disabled()
    page.click('#physical-fall-right');fall=page.evaluate('window.wheelbotLab.run(400)')
    assert fall['steps']==400 and not fall['failed'] and abs(fall['truth'][2])>1.2
    assert any('torso_visual' in [q['geom1'],q['geom2']] and 'floor' in [q['geom1'],q['geom2']] for q in fall['last']['allContacts']['pairs'])
    assert any(q['normalForceN']>1 for q in fall['last']['allContacts']['pairs'])
    page.locator('#view').screenshot(path=str(ROOT/'evidence/wheelbot_contact_fallen.png'))
    # A local control test is an explicit reset, not an implicit get-up claim.
    page.click('#physical-control');local=page.evaluate('window.wheelbotLab.run(100)')
    assert local['steps']==100 and not local['failed'] and abs(local['truth'][2])<.1
    page.locator('[data-design="base.massKg"]').fill('1.3')
    page.locator('[data-design="upper.lengthM"]').fill('0.30')
    page.locator('[data-design="motor.torqueLimitNm.0"]').fill('5')
    page.click('#physical-apply');page.wait_for_function('!window.wheelbotLab.getDesignEditor().busy && window.wheelbotLab.getDesignEditor().configuration.base.massKg===1.3',timeout=60000)
    edited=page.evaluate('window.wheelbotLab.getDesignEditor()')
    assert abs(edited['modelInfo']['totalMassKg']-1.49915)<1e-10
    assert edited['modelInfo']['torqueLimitsNm'][0]==5 and not edited['profileValid']
    assert page.locator('#physical-control').is_disabled()
    before=page.evaluate('JSON.stringify(window.wheelbotLab.getState())')
    rejected=page.evaluate("async()=>{const c=window.wheelbotLab.getDesignEditor().configuration;c.base.massKg=-1;try{await window.wheelbotLab.applyPhysicalDesign(c);return false;}catch(e){return true;}}")
    assert rejected and page.evaluate('JSON.stringify(window.wheelbotLab.getState())')==before
    oldprofile=json.loads((ROOT/'assets/wheelbot/profile.json').read_text())
    assert page.evaluate("async(p)=>{try{await window.wheelbotLab.importPhysicalProfile(p);return false;}catch(e){return true;}}",oldprofile)
    page.click('#physical-obstacle');obstacle=page.evaluate('window.wheelbotLab.run(400)')
    assert obstacle['steps']==400 and not obstacle['failed']
    assert any('obstacle' in [q['geom1'],q['geom2']] for q in obstacle['last']['allContacts']['pairs'])
    page.locator('#view').screenshot(path=str(ROOT/'evidence/wheelbot_contact_obstacle.png'))
    page.screenshot(path=str(ROOT/'evidence/wheelbot_contact_editor.png'),full_page=True)
    page.click('#physical-restore');restored=page.evaluate('window.wheelbotLab.getState()')
    assert not restored['configuredModel'] and restored['physics']['assetSha256']==original['physics']['assetSha256']
    assert not page.locator('#jump').is_disabled()
    run=page.evaluate('window.wheelbotLab.run(20)');assert run['steps']==20 and not run['failed']
    assert not errors,errors
    assert page.evaluate('document.documentElement.scrollWidth-innerWidth')<=1
    out={'schema':'wheelbot-contact-browser/v1','passed':True,'actualWasm':True,'fullBodyFallContinues':True,'actualPositiveGroundReaction':True,'obstacleCollision':True,'massLinkTorqueChangeApplied':True,'staleProfileRejected':True,'invalidConfigPreservesState':True,'localStandingProfileIndependent':True,'explicitRestoreWorks':True,'selfRightingDemonstrated':False,'pageErrors':errors}
    (ROOT/'evidence/wheelbot_contact_browser.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out));browser.close()
finally:server.shutdown()
