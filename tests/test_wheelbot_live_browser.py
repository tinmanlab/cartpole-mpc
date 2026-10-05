"""CI browser verification of the default compact robot, controls and wall-clock pacing."""
from pathlib import Path
import argparse,functools,http.server,json,threading,time
from playwright.sync_api import sync_playwright
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'test-results/live-browser';OUT.mkdir(parents=True,exist_ok=True)
p=argparse.ArgumentParser();p.add_argument('--url');args=p.parse_args();server=None
if not args.url:
 class Quiet(http.server.SimpleHTTPRequestHandler):
  def log_message(self,*a):pass
 server=http.server.ThreadingHTTPServer(('127.0.0.1',0),functools.partial(Quiet,directory=str(ROOT)))
 threading.Thread(target=server.serve_forever,daemon=True).start()
url=args.url or f'http://127.0.0.1:{server.server_port}/wheelbot.html';report={'passed':False,'url':url}
try:
 with sync_playwright() as p:
  browser=p.chromium.launch(headless=True);page=browser.new_page(viewport={'width':1440,'height':1000});errors=[]
  page.on('pageerror',lambda e:errors.append(str(e)));page.goto(url,wait_until='networkidle');page.wait_for_function('window.wheelbotLab?.ready && window.wheelbotLab.getState().liveActive',timeout=60000)
  state=lambda:page.evaluate('window.wheelbotLab.getState()')
  s=state();assert abs(s['truth'][4])>.9 and abs(s['truth'][4])<1.3 and not s['failed'];assert s['estimate'] is not None
  assert page.locator('.pipeline span').all_text_contents()==['Six noisy channels: five poses + wheel encoder rate','KF state estimate','LQR feedback','MuJoCo full contact']
  canvas=page.locator('#view').bounding_box();assert canvas['y']<300 and canvas['y']+canvas['height']<900
  assert page.locator('#play').is_visible() and page.locator('#live-left').is_visible()
  page.screenshot(path=str(OUT/'default.png'),full_page=False)
  page.locator('#physical-editor').evaluate('(e)=>e.open=true');after=page.locator('#view').bounding_box();assert abs(canvas['y']-after['y'])<1
  page.screenshot(path=str(OUT/'settings.png'),full_page=False);page.locator('#physical-editor').evaluate('(e)=>e.open=false')
  # Public runtime telemetry is measured; it is not an intended-rate label.
  page.click('#live-right');start=page.evaluate('({clock:performance.now(),steps:window.wheelbotLab.getState().steps})');page.wait_for_timeout(5000)
  stop=page.evaluate('({clock:performance.now(),state:window.wheelbotLab.getState(),performance:window.wheelbotLab.getPerformance()})')
  elapsed=(stop['clock']-start['clock'])/1000;sim=(stop['state']['steps']-start['steps'])*.01;rtf=sim/elapsed
  assert not stop['state']['failed'] and rtf>=.85,{'rtf':rtf,'perf':stop['performance']}
  assert abs(stop['state']['truth'][0]-.03)<.02 and stop['state']['goal']==.03
  page.evaluate('window.wheelbotLab.pause()');before=state();page.select_option('#goal','-0.03');now=state()
  assert now['steps']>=before['steps'] and abs(now['truth'][0]-before['truth'][0])<.005 and now['goal']==-.03
  page.wait_for_timeout(5000);moved=state();assert not moved['failed'] and abs(moved['truth'][0]+.03)<.02
  page.evaluate('window.wheelbotLab.pause()');frozen=state()['physics']['steps'];page.wait_for_timeout(100);assert state()['physics']['steps']==frozen
  page.click('#step');assert state()['physics']['steps']==frozen+5
  page.click('#reset');assert state()['steps']==0 and abs(state()['truth'][4]+1.1)<1e-8
  page.screenshot(path=str(OUT/'controlled-crouch.png'),full_page=False)
  for width,height in [(1440,900),(390,850)]:
   page.set_viewport_size({'width':width,'height':height});page.wait_for_timeout(100);assert page.evaluate('document.documentElement.scrollWidth-innerWidth')<=1
  assert not errors,errors
  report.update(passed=True,defaultCrouch=True,actualModel=s['physics']['assetSha256'],initialKneeFlexionDeg=abs(s['truth'][4])*180/3.141592653589793,viewportInitial=canvas,settingsDoNotPushViewport=True,setpointDoesNotReset=True,measuredWallSeconds=elapsed,simulatedSeconds=sim,realTimeFactor=rtf,performance=stop['performance'],pageErrors=errors,scope='Headless CI wall-clock sample, not hard-real-time or hardware validation.')
  browser.close()
finally:
 (OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n')
 if server:server.shutdown()
print(json.dumps(report))
