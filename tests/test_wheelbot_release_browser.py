"""Real browser release gates. CI only locally; --url uses a published page."""
from pathlib import Path
import argparse, functools, hashlib, http.server, json, threading
from playwright.sync_api import sync_playwright
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'test-results/wheelbot-browser'; OUT.mkdir(parents=True,exist_ok=True)
def json_response_handler(payload):
    # Playwright may pass both Route and Request; keep response data separate.
    def handler(route, request=None):
        route.fulfill(body=payload,content_type='application/json')
    return handler

ap=argparse.ArgumentParser(); ap.add_argument('--url',help='Full public wheelbot.html URL; starts no server'); args=ap.parse_args()
server=None
if not args.url:
    class Quiet(http.server.SimpleHTTPRequestHandler):
        def log_message(self,*args): pass
    server=http.server.ThreadingHTTPServer(('127.0.0.1',0),functools.partial(Quiet,directory=str(ROOT)))
    threading.Thread(target=server.serve_forever,daemon=True).start()
url=args.url or f'http://127.0.0.1:{server.server_port}/wheelbot.html'
report={'url':url,'cases':{},'assetHashes':{},'passed':False}
try:
    with sync_playwright() as p:
        browser=p.chromium.launch(headless=True)
        page=browser.new_page(viewport={'width':1440,'height':1100}); errors=[]
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto(url,wait_until='networkidle')
        page.wait_for_function('window.wheelbotLab?.ready && window.wheelbotLab.getDesignEditor()',timeout=60000)
        def state(): return page.evaluate('window.wheelbotLab.getState()')
        def run(n): return page.evaluate('(n)=>window.wheelbotLab.run(n)',n)
        def snap(name): page.locator('#view').screenshot(path=str(OUT/(name+'.png')))
        def save(name,s):
            report['cases'][name]=s
            assert not s['failed'],name
        # Published bytes can be matched to the reviewed checkout, without a server.
        for path in ['assets/wheelbot/wheelbot.xml','assets/wheelbot/contact_model.xml','assets/wheelbot/contact_tracking_profile.json','src/wheelbot_app.mjs']:
            response=page.request.get(url.rsplit('/',1)[0]+'/'+path); assert response.ok,path
            digest=hashlib.sha256(response.body()).hexdigest(); assert digest==hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),path
            report['assetHashes'][path]=digest
        original=state(); baseline=run(20); assert baseline['steps']==20; save('baseline',baseline)
        page.select_option('#mode','mpc_kf'); page.select_option('#goal','0.03')
        standing=run(300); assert standing['steps']==300 and abs(standing['truth'][0]-.03)<.02
        save('standing_mpc_goal',standing); snap('standing')
        page.click('#reset'); page.select_option('#mode','lqr_kf')
        page.evaluate('window.wheelbotLab.prepareJump(809)'); flight=run(100)
        assert flight['last']['geometry']['wheelContacts']==0; snap('jump_airborne')
        jump=run(300); assert jump['steps']==400 and jump['jumpResult']['passed'] and jump['jumpResult']['externalForceIsZero']; save('jump',jump)
        assert run(10)['physics']['steps']==jump['physics']['steps']
        page.click('#reset')
        for mode in ['lqr_kf','mpc_kf']:
            page.select_option('#mode',mode)
            for direction in [-1,1]:
                page.evaluate('(d)=>window.wheelbotLab.prepareForce(d)',direction)
                force=run(300); assert force['steps']==300 and force['forceResult']['normalPassed']; save(f'force_{mode}_{direction}',force)
                page.click('#reset')
        # Explicit visible model switch, followed by actual button/wall-time runs.
        page.click('#try-contact'); page.wait_for_function('window.wheelbotLab.getDesignEditor().trackingValid && !window.wheelbotLab.getDesignEditor().busy')
        assert state()['configuredModel'] and page.locator('#jump').is_disabled()
        assert 'Full-contact model' in page.locator('#active-model').inner_text()
        for side in ['left','right']:
            page.click('#physical-track-'+side)
            page.wait_for_function('window.wheelbotLab.getState().done',timeout=30000)
            s=state(); assert s['steps']==250 and s['elapsedSeconds']==.5 and s['trackingResult']['tracking_success'] and s['estimate'] is None
            save('lift_'+side,s); snap('lift_'+side)
            assert run(10)['physics']['steps']==s['physics']['steps']
        page.evaluate('window.wheelbotLab.prepareTracking(-1)'); page.click('#step')
        assert state()['steps']==1 and state()['elapsedSeconds']==.002
        page.click('#play'); page.wait_for_function('window.wheelbotLab.getState().steps>5'); page.click('#play')
        paused=state(); assert not paused['playing']; page.wait_for_timeout(100)
        assert state()['physics']['steps']==paused['physics']['steps']
        page.click('#reset'); assert state()['steps']==0 and not state().get('tracking',False)
        page.click('#physical-fall-right'); fall=run(400)
        assert fall['steps']==400 and abs(fall['truth'][2])>1.2
        assert any('torso_visual' in [q['geom1'],q['geom2']] and 'floor' in [q['geom1'],q['geom2']] and q['normalForceN']>1 for q in fall['last']['allContacts']['pairs'])
        save('fallen',fall); snap('fallen')
        for field,value in [('base.massKg','1.3'),('upper.lengthM','0.30'),('motor.torqueLimitNm.0','5')]:
            page.locator('[data-design="'+field+'"]').fill(value); page.locator('[data-design="'+field+'"]').press('Tab')
        page.click('#physical-apply'); page.wait_for_function('!window.wheelbotLab.getDesignEditor().busy && window.wheelbotLab.getDesignEditor().modelInfo.torqueLimitsNm[0]===5')
        info=page.evaluate('window.wheelbotLab.getDesignEditor()'); assert abs(info['modelInfo']['totalMassKg']-1.49915)<1e-10
        assert info['configuration']['upper']['lengthM']==.30 and not info['profileValid'] and not info['trackingValid']
        assert page.locator('#physical-track-left').is_disabled() and page.locator('#physical-control').is_disabled()
        assert page.evaluate('()=>{const before=JSON.stringify(window.wheelbotLab.getState());try{window.wheelbotLab.prepareTracking(1);return false}catch{return before===JSON.stringify(window.wheelbotLab.getState())}}')
        page.click('#physical-obstacle'); obstacle=run(400); save('obstacle',obstacle)
        assert any('obstacle' in [q['geom1'],q['geom2']] for q in obstacle['last']['allContacts']['pairs'])
        page.screenshot(path=str(OUT/'editor.png'),full_page=True)
        page.click('#physical-restore'); assert not state()['configuredModel'] and state()['physics']['assetSha256']==original['physics']['assetSha256']
        assert not page.locator('#jump').is_disabled(); assert run(20)['steps']==20
        # A stale optional source disables only its own feature.
        for filename,field,value,button in [('contact_tracking_profile.json','steps',249,'physical-track-left'),('jump_profile.json','baselineSha256','bad','jump')]:
            bad=json.loads((ROOT/'assets/wheelbot'/filename).read_text()); bad[field]=value
            pattern='**/'+filename
            page.route(pattern,json_response_handler(json.dumps(bad)))
            page.reload(wait_until='networkidle'); page.wait_for_function('window.wheelbotLab?.ready && window.wheelbotLab.getDesignEditor()')
            assert run(20)['steps']==20
            if filename.startswith('contact'):
                page.click('#try-contact'); page.wait_for_function('window.wheelbotLab.getState().configuredModel && !window.wheelbotLab.getDesignEditor().busy')
                assert not page.locator('#physical-fall-left').is_disabled()
            assert page.locator('#'+button).is_disabled(); page.unroute(pattern)
        for width in [1440,390]:
            page.set_viewport_size({'width':width,'height':900})
            assert page.evaluate('document.documentElement.scrollWidth-innerWidth')<=1
        assert not errors,errors
        report.update(passed=True,pageErrors=errors,actualButtonWalltimeLift=True)
        browser.close()
finally:
    (OUT/'release.json').write_text(json.dumps(report,indent=2)+'\n')
    if server: server.shutdown()
print(json.dumps({'passed':report['passed'],'cases':list(report['cases']),'url':url}))
