"""Independent compact jump closed loop using six noisy measurements.
Actual limits and contact transitions are checked, never a visual reset.
"""
from pathlib import Path
import hashlib,json
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1]
p=json.loads((ROOT/'assets/wheelbot/action_jump.json').read_text());base=json.loads((ROOT/'assets/wheelbot/live_profile.json').read_text())
assert p['assetSha256']==hashlib.sha256((ROOT/'assets/wheelbot/live_model.xml').read_bytes()).hexdigest()
m=mujoco.MjModel.from_xml_path(str(ROOT/'assets/wheelbot/live_model.xml'));d=mujoco.MjData(m)
refs=np.array(p['ref']);U=np.array(p['u']);K=np.array(p['K']);A=np.array(p['A']);B=np.array(p['B']);L=np.array(p['L']);ix=p['controlledIndices'];mi=p['measurementIndices'];H=np.eye(11)[[0,1,2,3,4,10]];limits=np.array(p['limitsNm']);results=[];raw=[]
for seed,offset,error in [(1,0,0),(7,.5,0),(42,-.5,0),(2026,0,.003),(17,0,-.003),(819,.25,.005)]:
 rng=seed
 def uniform():
  global rng
  rng=(1664525*rng+1013904223)&0xffffffff;return(rng+.5)/4294967296
 def measure(x):return x[mi]+np.array(p['measurementSigma'])*np.array([np.sqrt(-2*np.log(uniform()))*np.cos(2*np.pi*uniform())for _ in mi])
 r=refs.copy();r[:,0]+=offset;x=r[0].copy();x[2]+=error;initial=x.copy();first=measure(x);estimate=np.r_[first[:5],np.zeros(6)];history=[];flight=longest=0;pen=exc=0.;peak=np.zeros(3);failure=False
 for k in range(p['steps']):
  terminal=k>=200
  current_ref=np.r_[base['qref'][:5],np.zeros(6)] if terminal else r[k,ix].copy()
  if terminal:current_ref[0]=offset
  gain=np.array(base['K'])if terminal else K[k];feed=np.array(base['uref'])if terminal else U[k]
  u=np.clip(feed-gain@(estimate-current_ref),-limits,limits);before=x.copy();mujoco.mj_resetData(m,d);d.qpos[:]=x[:6];d.qvel[:]=x[6:];d.ctrl[:]=u
  for _ in range(5):
   mujoco.mj_step(m,d);pen=max(pen,max([0.]+[-float(c.dist)for c in d.contact]));exc=max(exc,0.,abs(d.qpos[3])-1.26,abs(d.qpos[4])-2.51)
  x=np.r_[d.qpos,d.qvel].copy();measurement=measure(x)
  pred=current_ref+np.array(base['A'])@(estimate-current_ref)+np.array(base['B'])@(u-feed)if terminal else r[k+1,ix]+A[k]@(estimate-r[k,ix])+B[k]@(u-U[k])
  gain_l=np.array(base['L'])if terminal else L[k];estimate=pred+gain_l@(measurement-H@pred)
  mujoco.mj_forward(m,d);wheel=m.geom('wheel_visual').id;contacts=sum(wheel in (c.geom1,c.geom2)for c in d.contact);clear=float(d.geom_xpos[wheel,2]-.05);flight=flight+1 if contacts==0 and clear>.005 else 0;longest=max(longest,flight);peak=np.maximum(peak,np.abs(u))
  history.append({'before':before.tolist(),'after':x.tolist(),'u':u.tolist(),'measurement':measurement.tolist(),'estimate':estimate.tolist(),'clearance':clear,'contacts':contacts,'comZ':float(d.subtree_com[1,2])})
  if not np.isfinite(x).all()or abs(x[2]-r[0,2])>.6 or abs(x[0]-offset)>1 or x[1]<.15:failure=True;break
 tail=np.array([h['after']for h in history[-50:]]);errors=np.max(np.abs(tail[:,:5]-r[0,:5]),axis=0);speed=float(np.max(np.abs(tail[:,6:])));success=bool(len(history)==600 and not failure and longest>=3 and pen<=.01 and exc<=.02 and errors[0]<.03 and errors[2]<.04 and max(errors[3:5])<.06 and speed<.3)
 result={'seed':seed,'xOffset':offset,'initialPitchError':error,'passed':success,'steps':len(history),'flightSeconds':longest*.01,'maxPenetration':pen,'maxJointExcursion':exc,'tailErrors':errors.tolist(),'tailMaxRate':speed,'peakTorque':peak.tolist(),'maxWheelClearance':max(h['clearance']for h in history)}
 results.append(result);raw.append({'case':result,'initial':initial.tolist(),'trace':history});print(json.dumps(result),flush=True)
report={'schema':'wheelbot-action-jump-reference/v1','profileSha256':hashlib.sha256((ROOT/'assets/wheelbot/action_jump.json').read_bytes()).hexdigest(),'modelSha256':p['assetSha256'],'passed':sum(r['passed']for r in results),'trials':len(results),'results':results,'scope':'Same compact robot; six noisy measurement channels; no true-state feedback/force/reset. Known bounded entry conditions, no arbitrary jump/getup or hardware claim.'}
(ROOT/'evidence/wheelbot_action_jump.json').write_text(json.dumps(report,indent=2)+'\n');(ROOT/'test-results/action-jump-reference.json').write_text(json.dumps({'report':report,'raw':raw})+'\n')
assert report['passed']==report['trials'],report
