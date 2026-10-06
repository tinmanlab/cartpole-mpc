"""Actual keyboard/render acceptance; use the normal CI shared HTTP server only."""
import argparse, hashlib, json, math
from pathlib import Path
from playwright.sync_api import sync_playwright
ap=argparse.ArgumentParser();ap.add_argument('--url',required=True);args=ap.parse_args()
out=Path('test-results');out.mkdir(exist_ok=True)
report={'passed':False,'flows':[],'errors':[]}
with sync_playwright() as p:
 browser=p.chromium.launch(headless=True)
 page=browser.new_page(viewport={'width':1440,'height':900},device_scale_factor=1)
 page.on('pageerror',lambda e:report['errors'].append(str(e)))
 def state():return page.evaluate('wheelbotGame.getState()')
 def run(n):return page.evaluate('(n)=>wheelbotGame.run(n)',n)
 def reset():
  page.click('#reset');page.wait_for_function('wheelbotGame.ready');run(200)
 def record(name,data=None):report['flows'].append({'name':name,'passed':True,'data':data})
 def shot(name):page.screenshot(path=str(out/('wheelbot-game-'+name+'.png')))
 try:
  page.goto(args.url);page.wait_for_function('window.wheelbotGame?.ready',timeout=60000)
  assert page.locator('canvas').count()==1 and not page.locator('#help').evaluate('(e)=>e.open')
  assert page.locator('#view').evaluate('(e)=>document.activeElement===e')
  for text in ['이동','높이','기울기','Space']:assert text in page.locator('#controls').inner_text()
  assert page.locator('#course option').count()==4
  # Pixel rectangles: the canvas fills the viewport and controls leave the robot region clear.
  bounds=page.evaluate('()=>Object.fromEntries(["view","controls","help","failure"].map(id=>{const r=document.getElementById(id).getBoundingClientRect();return [id,{x:r.x,y:r.y,w:r.width,h:r.height}]}))')
  assert bounds['view']=={'x':0,'y':0,'w':1440,'h':900}
  assert bounds['controls']['h']<180 and bounds['controls']['y']>720
  assert bounds['help']['y']+bounds['help']['h']<250
  assert page.evaluate('()=>[...document.querySelectorAll(".hud .cluster")].every(e=>e.getBoundingClientRect().bottom<250)')
  pixels=page.evaluate("""()=>{const c=document.querySelector('#view'),ctx=c.getContext('2d'),w=c.width,h=c.height;const p=ctx.getImageData(Math.floor(w*.4),Math.floor(h*.4),Math.floor(w*.2),Math.floor(h*.42)).data;let ink=0;for(let i=0;i<p.length;i+=4)if(p[i]<100&&p[i+1]<160&&p[i+2]<200)ink++;return ink;}""")
  assert pixels>100,'Actual robot pixels missing from the unobstructed viewport'
  shot('default');record('layout',{'bounds':bounds,'robotInkPixels':pixels})
  # Actual wall-clock running, without the accelerated helper.
  page.wait_for_timeout(3300)
  perf=page.evaluate('wheelbotGame.getPerformance()');report['wallPerformance']=perf
  assert perf['wallSeconds']>=3 and perf['realTimeFactor']>=.85,perf
  assert perf['samples']>0 and perf['controlP95Ms']>0
  assert all(math.isfinite(x) for x in state()['truth']) and not state()['failed']
  record('wall-clock',perf)
  before=state();page.keyboard.down('KeyD');page.wait_for_timeout(1200);moving=state()
  assert moving['truth'][6]>.1 and moving['truth'][0]>before['truth'][0]+.05,moving
  page.keyboard.up('KeyD');run(250);assert abs(state()['truth'][6])<.04
  record('horizontal and release',{'before':before,'moving':moving,'stopped':state()})
  reset();z=state()['truth'][1]
  page.keyboard.down('KeyW');run(70);page.keyboard.up('KeyW');run(80);assert state()['truth'][1]>z+.02
  page.keyboard.down('KeyS');run(100);page.keyboard.up('KeyS');run(80);assert state()['truth'][1]<z+.01
  page.keyboard.down('KeyE');run(20);page.keyboard.up('KeyE');run(100);assert state()['truth'][2]>.03
  page.keyboard.down('KeyQ');run(40);page.keyboard.up('KeyQ');run(100);assert state()['truth'][2]<-.03
  record('height and pitch',state())
  reset();page.keyboard.press('KeyP');n=state()['steps'];page.keyboard.down('Space');page.wait_for_timeout(150)
  assert state()['steps']==n and not state()['charge']['active']
  page.keyboard.press('KeyP');page.keyboard.up('Space');assert state()['phase']!='jump'
  page.keyboard.down('Space');run(10);page.focus('#course');page.keyboard.up('Space');assert not state()['charge']['active'] and state()['phase']!='jump'
  page.focus('#view');page.keyboard.down('Space');run(10);page.evaluate('window.dispatchEvent(new Event("blur"))');page.keyboard.up('Space');assert not state()['charge']['active'] and state()['phase']!='jump'
  page.keyboard.press('KeyP');page.keyboard.down('Space');run(10);page.keyboard.press('Escape');page.keyboard.up('Space');assert state()['phase']!='jump'
  page.keyboard.press('Escape');record('pause, editable focus, blur and Escape cancellation')
  heights=[]
  for moving in [False,True]:
   for duration in ([10,50,100] if not moving else [10,100]):
    reset()
    if moving:page.keyboard.down('KeyD');run(200)
    page.keyboard.down('Space');run(duration)
    charged=state();page.keyboard.down('Space');assert state()['charge']['seconds']>=charged['charge']['seconds']
    assert abs(charged['charge']['seconds']-min(1,duration*.01))<.12
    before=state();page.keyboard.up('Space');released=state()
    assert released['phase']=='jump',released['status']
    assert abs(released['truth'][0]-before['truth'][0])<.03
    if released['steps']==before['steps']:assert released['truth']==before['truth'] and released['estimate']==before['estimate']
    if moving:assert released['truth'][6]>.1
    airborne=False;min_v=10;flight_com_v=[]
    prior_com=page.evaluate('wheelbotGame.getWorld().centerOfMass[0]');prior_steps=state()['steps'];prior_air=False
    for _ in range(80):
     s=run(5);min_v=min(min_v,s['truth'][6])
     com=page.evaluate('wheelbotGame.getWorld().centerOfMass[0]');air=s['last']['physical']['flightSamples']
     all_air=all(z['air'] for z in air)
     if all_air and prior_air:flight_com_v.append((com-prior_com)/((s['steps']-prior_steps)*.01))
     prior_com,prior_steps,prior_air=com,s['steps'],all_air
     if s['last']['physical']['maximumWheelClearanceM']>.01 and not airborne:
      airborne=True;shot('flight-moving' if moving else 'flight')
     assert not s['failed'],s
    assert airborne and s['jump']['flight'] and s['jump']['landed'] and s['jump']['success'],s
    if moving:
     # An articulated base can recoil during landing; do not confuse this with
     # a stopped launch or loss of forward flight momentum. Retain the minimum
     # base velocity in evidence, and check actual airborne COM progression.
     assert len(flight_com_v)>=1 and min(flight_com_v)>.05,flight_com_v
     assert s['truth'][0]>before['truth'][0]+.4 and s['truth'][6]>.15
     page.keyboard.up('KeyD')
    else:heights.append(s['jump']['maxClearance'])
    record(f'jump moving={moving} hold={duration/100}',{'charge':charged['charge'],'before':before,'released':released,'landed':s,'minimumBaseVxIncludingLanding':min_v,'airborneCOMVx':flight_com_v})
  assert heights[0]<heights[1]<heights[2],heights
  robot_hash=hashlib.sha256(Path('assets/wheelbot/live_model.xml').read_bytes()).hexdigest()
  for level,count in [('flat',0),('obstacles',4),('ramp',3),('uneven',16)]:
   page.select_option('#course',level)
   page.wait_for_function('(level)=>wheelbotGame.ready&&wheelbotGame.getWorld().manifest.id===level',arg=level,timeout=60000)
   world=page.evaluate('wheelbotGame.getWorld()')
   assert world['robotAssetSha256']==robot_hash
   assert len([g for g in world['geoms'] if g['name'].startswith('terrain_')])==count
   assert (world['assetSha256']==robot_hash)==(level=='flat')
   assert page.locator('#view').evaluate('(e)=>document.activeElement===e')
   shot('course-'+level);record('compiled course '+level,world)
  assert not report['errors'],report['errors']
  report['passed']=True
 except Exception as e:
  report['errors'].append(repr(e));shot('failure');raise
 finally:
  (out/'wheelbot-game-browser.json').write_text(json.dumps(report,indent=2))
  browser.close()

print(json.dumps({"passed":report["passed"],"flows":len(report["flows"]),"performance":report.get("wallPerformance"),"errors":report["errors"],"scope":"Actual keyboard motion, stationary/moving charged hops, focus/pause handling and compiled collision courses; no universal terrain traversal claim."}))
