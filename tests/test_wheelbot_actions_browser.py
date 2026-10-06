"""User-facing pose, physical disturbances and compact-model jump on one page.
Run in ordinary CI; --url checks a deployed page without creating a server.
"""
from pathlib import Path
import argparse,functools,hashlib,http.server,json,threading
from playwright.sync_api import sync_playwright
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'test-results/actions-browser';OUT.mkdir(parents=True,exist_ok=True)
ap=argparse.ArgumentParser();ap.add_argument('--url');args=ap.parse_args();server=None
if not args.url:
 class Quiet(http.server.SimpleHTTPRequestHandler):
  def log_message(self,*args):pass
 server=http.server.ThreadingHTTPServer(('127.0.0.1',0),functools.partial(Quiet,directory=str(ROOT)))
 threading.Thread(target=server.serve_forever,daemon=True).start()
url=args.url or f'http://127.0.0.1:{server.server_port}/wheelbot.html';report={'passed':False,'url':url}
try:
 with sync_playwright() as p:
  browser=p.chromium.launch(headless=True);page=browser.new_page(viewport={'width':1440,'height':1000});errors=[]
  page.on('pageerror',lambda error:errors.append(str(error)))
  page.goto(url,wait_until='networkidle');page.wait_for_function('window.wheelbotLab?.ready && window.wheelbotLab.getState().actions',timeout=60000)
  state=lambda:page.evaluate('window.wheelbotLab.getState()')
  run=lambda n:page.evaluate('(n)=>window.wheelbotLab.run(n)',n)
  assert not page.locator('#advanced-lab').evaluate('(e)=>e.open')
  assert page.locator('#view').bounding_box()['y']<120
  for name in ['target-x','target-z','target-pitch','action-jump','disturb-left','disturb-right','reset']:
   assert page.locator('#'+name).is_visible(),name
   box=page.locator('#'+name).bounding_box();assert box['y']+box['height']<950,(name,box)
  assert page.locator('#action-recover').is_disabled()
  assert page.locator('#target-x').get_attribute('min')=='-1' and page.locator('#target-x').get_attribute('max')=='1'
  page.screenshot(path=str(OUT/'simple-default.png'),full_page=False)
  # The goal setter is synchronous, so compare before/after before RAF can run.
  changed=page.evaluate('()=>{wheelbotLab.pause();const before=wheelbotLab.getState();wheelbotLab.setLiveTarget({x:.75,z:.40,pitch:-Math.PI/30});const after=wheelbotLab.getState();wheelbotLab.pause();return {before,after}}')
  for key in ['truth','estimate','steps']:assert changed['before'][key]==changed['after'][key],key
  pose=run(2000);assert not pose['failed'] and abs(pose['truth'][0]-.75)<.03 and abs(pose['truth'][1]-.40)<.01 and abs(pose['truth'][2]+3.141592653589793/30)<.04
  page.screenshot(path=str(OUT/'pose-target.png'),full_page=False)
  page.click('#target-home');run(2000)
  # Actual continuous browser play, not only an API loop.
  page.click('#play');start=page.evaluate('({t:performance.now(),steps:wheelbotLab.getState().steps})');page.wait_for_timeout(5000)
  finish=page.evaluate('({t:performance.now(),s:wheelbotLab.getState(),performance:wheelbotLab.getPerformance()})')
  rtf=(finish['s']['steps']-start['steps'])*.01/((finish['t']-start['t'])/1000)
  assert rtf>=.85 and not finish['s']['failed'],{'rtf':rtf,'performance':finish['performance']}
  page.evaluate('wheelbotLab.pause()');fixed=state()['physics']['steps'];page.wait_for_timeout(100);assert state()['physics']['steps']==fixed
  # Jump on this same robot, at the actual current state, with no teleport.
  before=state();page.click('#action-jump')
  page.wait_for_function('wheelbotLab.getState().phase==="jumping" && wheelbotLab.getState().last?.contact.wheelContacts===0',timeout=40000)
  airborne=state();assert airborne['physics']['assetSha256']==before['physics']['assetSha256'] and airborne['steps']>before['steps']
  page.screenshot(path=str(OUT/'compact-jump.png'),full_page=False)
  page.wait_for_function('wheelbotLab.getState().lastJumpResult!==null',timeout=40000)
  jumped=state();assert jumped['lastJumpResult']['passed'],jumped['lastJumpResult']
  assert jumped['lastJumpResult']['maximumWheelClearanceM']>.03 and jumped['lastJumpResult']['flightSeconds']>=.03
  page.evaluate('wheelbotLab.pause()')
  # Six declared medium forces/moments; torque is not described as a force.
  disturbances=[]
  for kind in ['push','gust','twist']:
   for direction in [-1,1]:
    data=page.evaluate('(v)=>{const a=wheelbotLab.getState();wheelbotLab.disturb(v);const b=wheelbotLab.getState();wheelbotLab.pause();const first=wheelbotLab.run(1);return {a,b,first}}',{'kind':kind,'direction':direction,'strength':'medium'})
    assert data['a']['truth']==data['b']['truth'] and data['a']['estimate']==data['b']['estimate']
    wrench=data['first']['last']['externalWrench'];assert wrench[2 if kind=='twist' else 0]*direction>0
    restored=run(400);assert restored['last']['externalWrench']==[0,0,0] and not restored['failed']
    disturbances.append({'kind':kind,'direction':direction,'firstWrench':wrench,'remainingBalance':not restored['failed']})
  # An actual strong pulse can cause a fall. Do not turn reset into fake get-up.
  page.click('#reset');page.evaluate('wheelbotLab.setLiveTarget({x:1,z:.36,pitch:Math.PI/18});wheelbotLab.pause()');run(1200)
  page.evaluate('wheelbotLab.disturb({kind:"twist",direction:1,strength:"strong"});wheelbotLab.pause()');fallen=run(1500)
  assert fallen['failed'] and fallen['phase'] in ['falling','fallen'],fallen['phase']
  assert fallen['observerValid'] is False and fallen['last']['u']==[0,0,0]
  steps=fallen['physics']['steps'];continued=run(50);assert continued['physics']['steps']==steps+250
  refused=page.evaluate('()=>{const before=JSON.stringify(wheelbotLab.getState());try{wheelbotLab.recover();return false;}catch{return before===JSON.stringify(wheelbotLab.getState());}}')
  assert refused and page.locator('#action-recover').is_disabled()
  page.screenshot(path=str(OUT/'physical-fall.png'),full_page=False)
  page.click('#reset');assert state()['steps']==0 and not state()['failed']
  # Missing/invalid optional jump data must not stop the baseline pose controller.
  payload={'schema':'wheelbot-action-jump/v1','steps':1}
  def intercept(route,request=None):route.fulfill(body=json.dumps(payload),content_type='application/json')
  page.route('**/action_jump.json',intercept);page.reload(wait_until='networkidle');page.wait_for_function('wheelbotLab?.ready && wheelbotLab.getState().actions')
  assert page.locator('#action-jump').is_disabled() and not page.locator('#target-x').is_disabled()
  assert not run(100)['failed'];page.unroute('**/action_jump.json')
  assert page.evaluate('document.documentElement.scrollWidth-innerWidth')<=1 and not errors,errors
  report.update(passed=True,modelSha256=pose['physics']['assetSha256'],poseTarget={'x':.75,'z':.40,'pitch':-3.141592653589793/30},poseActual=pose['truth'][:3],statePreservingTargets=True,actualWallRealTimeFactor=rtf,performance=finish['performance'],sameRobotJump=jumped['lastJumpResult'],disturbances=disturbances,fallenContinuesPhysics=True,recoveryImplemented=False,resetNotRecovery=True,invalidJumpIsolated=True,pageErrors=errors,scope='Planar known-model scenarios and measured headless-browser pacing, not arbitrary physical recovery, hardware or hard realtime.')
  browser.close()
finally:
 (OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n')
 if server:server.shutdown()
print(json.dumps(report))
