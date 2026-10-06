"""Run in normal CI only: python tests/test_wheelbot_paths_browser.py --url URL."""
import argparse,json
from pathlib import Path
from playwright.sync_api import sync_playwright
ap=argparse.ArgumentParser();ap.add_argument('--url',required=True);args=ap.parse_args()
with sync_playwright() as p:
 browser=p.chromium.launch(headless=True)
 page=browser.new_page(viewport={'width':1100,'height':900},device_scale_factor=2)
 errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
 page.goto(args.url);page.wait_for_function('window.wheelbotLab?.ready')
 page.evaluate('wheelbotLab.run(30)')
 assert page.evaluate('wheelbotLab.getPerformance().fullLoop.active') is False
 box=page.locator('#view').bounding_box();assert box
 assert page.locator('#robot-model').count()==0
 assert page.evaluate('wheelbotLab.getModelHash()')==__import__('hashlib').sha256(Path('assets/wheelbot/live_model.xml').read_bytes()).hexdigest()
 before=page.evaluate('wheelbotLab.getState()')
 def screen(x,z):
  # Use actual bounding box/DPR canvas mapping, independently of pointer events.
  return page.evaluate('([x,z])=>{const c=document.querySelector("#view"),r=c.getBoundingClientRect(),s=Math.min((c.height-60)/.72,c.width/2.6);return [r.left+(c.width/2+x*s)*r.width/c.width,r.top+(c.height-43-z*s)*r.height/c.height]}',[x,z])
 page.mouse.click(*screen(.1,.45));page.wait_for_function('wheelbotLab.getState().path?.requested?.length===1')
 after=page.evaluate('wheelbotLab.getState()');assert after['truth']==before['truth'] and after['estimate']==before['estimate']
 page.mouse.move(*screen(0,.45));page.mouse.down()
 for point in [(0.03,.46),(.07,.43),(.1,.45)]:page.mouse.move(*screen(*point),steps=2)
 page.mouse.up();page.wait_for_function('wheelbotLab.getState().path?.requested?.length>1')
 assert page.evaluate('wheelbotLab.getState().path.requested.at(-1).x')>.09
 page.select_option('#target-action','jump');page.mouse.click(*screen(.1,.65));page.wait_for_function('wheelbotLab.getState().path?.available===false')
 assert '사용 불가' in page.locator('#status').inner_text()
 page.select_option('#target-action','follow');page.mouse.move(*screen(.1,.45));page.mouse.down();page.dispatch_event('#view','pointercancel',{'pointerId':1});page.mouse.up()
 resume_steps=page.evaluate('wheelbotLab.getState().steps')
 page.click('#play');page.wait_for_function('(n)=>wheelbotLab.getState().steps>n+10',arg=resume_steps);page.click('#play')
 assert not page.evaluate('wheelbotLab.getState().failed')
 # Real underfloor click, explicit reset, force buttons and bounded run API.
 page.select_option('#target-mode','wheel')
 page.mouse.click(*screen(.05,-.03));page.wait_for_function('wheelbotLab.getState().path?.requested?.at(-1).z<0')
 projection=page.evaluate('wheelbotLab.getState().path')
 assert projection['reason'] and (not projection['available'] or abs(projection['accepted'][-1]['z']-.05)<.001)
 for kind in ['push','gust','twist']:
  page.click('#reset');page.evaluate('wheelbotLab.run(30)')
  page.select_option('#disturb-kind',kind);page.select_option('#disturb-strength','light')
  before=page.evaluate('wheelbotLab.getState()');page.click('#disturb-right')
  now=page.evaluate('wheelbotLab.getState()');assert now['truth']==before['truth'] and now['estimate']==before['estimate']
  after=page.evaluate('wheelbotLab.run(1)');assert any(after['last']['externalWrench'])
  assert page.evaluate('wheelbotLab.run(30).disturbance') is None
 page.click('#reset');assert page.evaluate('wheelbotLab.getState().steps')==0
 assert page.evaluate('()=>{try{wheelbotLab.run(Infinity);return false}catch{return true}}')
 Path('test-results').mkdir(exist_ok=True)
 Path('test-results/wheelbot-pointer-browser.json').write_text(json.dumps({'state':page.evaluate('wheelbotLab.getState()'),'timing':page.evaluate('wheelbotLab.getPerformance()'),'modelHash':page.evaluate('wheelbotLab.getModelHash()')}))
 page.screenshot(path='test-results/wheelbot-pointer-browser.png',full_page=True)
 assert 'WASM' in page.locator('#versions').inner_text()
 assert page.locator('#jump-status').inner_text()
 # Live clock must progress during asynchronous preview.
 page.evaluate('wheelbotLab.run(100)');n=page.evaluate('wheelbotLab.getState().steps');page.click('#play')
 page.evaluate('void wheelbotLab.requestPath([{x:.1,z:.44}],{mode:"base",action:"follow"})')
 page.wait_for_function('(n)=>wheelbotLab.isPlanning() && wheelbotLab.getState().steps>n+3',arg=n)
 page.wait_for_function('!wheelbotLab.isPlanning()');page.wait_for_function('(n)=>wheelbotLab.getState().steps>n+30',arg=n)
 Path('test-results/wheelbot-live-timing.json').write_text(json.dumps(page.evaluate('wheelbotLab.getPerformance()')));page.click('#play')
 # Frozen seed 7: explicit ordinary low pose, bounded settling, then pointer jump.
 page.evaluate('wheelbotLab.pause()');page.click('#reset')
 page.evaluate('wheelbotLab.run(100)');page.click('#low-target')
 page.wait_for_function('!wheelbotLab.isPlanning() && wheelbotLab.getState().path?.available===true')
 page.evaluate('wheelbotLab.run(800)')
 for _ in range(300):
  if page.evaluate('wheelbotLab.getJumpAdmission().ready'):break
  page.evaluate('wheelbotLab.run(1)')
 assert page.evaluate('wheelbotLab.getJumpAdmission().ready'),page.evaluate('wheelbotLab.getJumpAdmission()')
 before=page.evaluate('wheelbotLab.getState()')
 page.select_option('#target-mode','wheel');page.select_option('#target-action','jump')
 page.mouse.click(*screen(.03,.09));page.wait_for_function('!wheelbotLab.isPlanning()')
 after=page.evaluate('wheelbotLab.getState()');assert after['truth']==before['truth'] and after['estimate']==before['estimate']
 assert after['path']['available'] and after['phase']=='jump',after['actionStatus']
 after=page.evaluate('wheelbotLab.run(600)')
 assert not after['failed'] and after['phase']=='standing' and after['jumpOutcome']['success'],after
 assert after['jumpOutcome']['flight'] and after['jumpOutcome']['landed'] and after['jumpOutcome']['maxClearance']>=.02
 assert not page.evaluate('wheelbotLab.run(200).failed')
 page.screenshot(path='test-results/wheelbot-small-jump.png',full_page=True)
 # Optional corrupt jump must preserve ordinary balance.
 bad_jump=json.loads(Path('assets/wheelbot/target_jump.json').read_text());bad_jump['nativeVersion']='mismatch'
 page.route('**/target_jump.json',lambda route:route.fulfill(json=bad_jump));page.reload();page.wait_for_function('window.wheelbotLab?.ready');page.wait_for_function('document.querySelector("#jump-status").textContent.includes("mismatch")');assert not page.evaluate('wheelbotLab.run(30).failed')
 # Stale atlas fails closed without ever loading a historical model/jump.
 atlas=json.loads(Path('assets/wheelbot/pose_profiles.json').read_text());atlas['assetSha256']='stale'
 page.route('**/pose_profiles.json',lambda route:route.fulfill(json=atlas));page.reload();page.wait_for_function('document.querySelector("#status").textContent.includes("rejected")')
 # Mismatched atlas version must reject the action runtime.
 atlas['assetSha256']=__import__('hashlib').sha256(Path('assets/wheelbot/live_model.xml').read_bytes()).hexdigest();atlas['versions']['mujoco']='mismatch'
 page.reload();page.wait_for_function('document.querySelector("#status").textContent.includes("version mismatch")')
 assert not errors,errors
 browser.close()
print('Pointer click/drag/cancel and stale-profile checks passed')
