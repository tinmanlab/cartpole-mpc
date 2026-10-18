"""Shared canvas integration. Run only in normal CI; local sockets are forbidden."""
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
  browser=p.chromium.launch(headless=True);page=browser.new_page();errors=[]
  page.on('pageerror',lambda e:errors.append(str(e)))
  page.goto(f'http://127.0.0.1:{server.server_port}/wheelbot.html',wait_until='networkidle')
  page.wait_for_function('window.wheelbotLab?.getDesignEditor()')
  page.locator('#physical-editor').evaluate('(e)=>e.open=true')
  page.click('#physical-apply');page.wait_for_function('window.wheelbotLab.getDesignEditor().trackingValid')
  for side in ['left','right']:
   page.click('#physical-track-'+side)
   page.wait_for_function('window.wheelbotLab.getState().done',timeout=30000)
   s=page.evaluate('window.wheelbotLab.getState()')
   assert s['steps']==250 and s['trackingResult']['tracking_success'] and s['elapsedSeconds']==.5
   assert not s['jumpActive'] and not s['forceActive'] and s['estimate'] is None
   assert 'exact-state research' in page.locator('#physical-tracking-status').inner_text()
  page.evaluate('window.wheelbotLab.prepareTracking(-1)')
  page.click('#step');s=page.evaluate('window.wheelbotLab.getState()');assert s['steps']==1 and s['elapsedSeconds']==.002
  page.click('#play');page.click('#play');page.evaluate('window.wheelbotLab.run(250)')
  page.locator('#view').screenshot(path=str(ROOT/'evidence/wheelbot_contact_tracking.png'))
  page.locator('[data-design="base.massKg"]').fill('1.3');page.click('#physical-apply')
  page.wait_for_function('!window.wheelbotLab.getDesignEditor().busy')
  assert page.locator('#physical-track-left').is_disabled()
  before=page.evaluate('JSON.stringify(window.wheelbotLab.getState())')
  assert page.evaluate('()=>{try{window.wheelbotLab.prepareTracking(1);return false}catch{return true}}')
  assert page.evaluate('JSON.stringify(window.wheelbotLab.getState())')==before
  assert not page.locator('#physical-fall-left').is_disabled()
  page.click('#physical-restore');assert page.locator('#physical-track-right').is_disabled()
  assert not page.evaluate('window.wheelbotLab.getState().configuredModel')
  # Corrupt/expired profile disables only tracking on a fresh default model.
  for value in [249,0]:
   profile=json.loads((ROOT/'assets/wheelbot/contact_tracking_profile.json').read_text());profile['steps']=value
   page.route('**/contact_tracking_profile.json',lambda route,body=json.dumps(profile):route.fulfill(body=body,content_type='application/json'))
   page.reload(wait_until='networkidle');page.wait_for_function('window.wheelbotLab?.getDesignEditor()')
   page.locator('#physical-editor').evaluate('(e)=>e.open=true');page.click('#physical-apply')
   page.wait_for_function('window.wheelbotLab.getState().configuredModel && !window.wheelbotLab.getDesignEditor().busy')
   assert page.locator('#physical-track-left').is_disabled() and not page.locator('#physical-fall-left').is_disabled()
   page.unroute('**/contact_tracking_profile.json')
  assert not errors,errors
  browser.close()
finally:server.shutdown()
