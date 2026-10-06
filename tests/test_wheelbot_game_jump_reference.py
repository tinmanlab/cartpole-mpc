"""Independent native moving-reference/KF rollout from versioned assets only."""
from pathlib import Path
import json, hashlib
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1]
p=json.loads((ROOT/'assets/wheelbot/target_jump.json').read_text())
assert mujoco.__version__==mujoco.mj_versionString()==p['nativeVersion']=='3.15.0'
assert hashlib.sha256((ROOT/'assets/wheelbot/live_model.xml').read_bytes()).hexdigest()==p['assetSha256']
m=mujoco.MjModel.from_xml_path(str(ROOT/'assets/wheelbot/live_model.xml'));d=mujoco.MjData(m);wheel=m.geom('wheel_visual').id
ix=p['controlledIndices'];mi=p['measurementIndices'];results=[];traces=[]
for seed in [7,42]:
 for vx in [0,-.1,.1,-.2,.2]:
  for id in [1,5]:
   pr=next(q for q in p['profiles'] if q['id']==id);rng=seed
   def uniform():
    global rng
    rng=(1664525*rng+1013904223)&0xffffffff;return (rng+.5)/4294967296
   def measure(x):return x[mi]+np.array(p['measurementSigma'])*np.array([np.sqrt(-2*np.log(uniform()))*np.cos(2*np.pi*uniform()) for _ in mi])
   x=np.array(pr['entry']);x[0]=1.4;x[6]=vx;x[11]=vx/.05
   estimate=np.r_[measure(x)[:5],vx,0,0,0,0,vx/.05];origin=estimate[0];initial=x.copy();initialEstimate=estimate.copy();history=[];flight=longest=pen=exc=spin=clearance=0.;landed=False;wasAir=False
   for k in range(400):
    terminal=k>=140;s=pr['terminal'] if terminal else pr
    def reference(n):
     r=np.r_[s['qref'][:5],np.zeros(6)] if terminal else np.array(pr['ref'][n])[ix].copy()
     r[0]+=origin+vx*n*.01;r[5]+=vx;r[10]+=vx/.05;return r
    r=reference(k);nr=reference(k+1);feed=np.array(s['uref'] if terminal else s['u'][k]);feed[2]+=.015*vx/.05
    mats={a:np.array(s[a] if terminal else s[a][k]) for a in ['A','B','K','L']}
    u=np.clip(feed-mats['K']@(estimate-r),-np.array(p['limitsNm']),p['limitsNm'])
    mujoco.mj_resetData(m,d);d.qpos[:]=x[:6];d.qvel[:]=x[6:];d.ctrl[:]=u
    for _ in range(5):
     mujoco.mj_step(m,d);mujoco.mj_forward(m,d);x=np.r_[d.qpos,d.qvel].copy();h=float(d.geom_xpos[wheel,2]-.05);contacts=any(wheel in (c.geom1,c.geom2) for c in d.contact);air=not contacts and h>.005
     flight=flight+.002 if air else 0;longest=max(longest,flight);clearance=max(clearance,h)
     if air:wasAir=True;spin=max(spin,abs(sum(x[8:12])))
     if wasAir and contacts:landed=True
     pen=max(pen,max([0.]+[-float(c.dist) for c in d.contact]));exc=max(exc,abs(x[3])-1.26,abs(x[4])-2.51)
     assert not d.qfrc_applied.any() and not d.xfrc_applied.any()
    y=measure(x);pred=nr+mats['A']@(estimate-r)+mats['B']@(u-feed);estimate=pred+mats['L']@(y-pred[[0,1,2,3,4,10]])
    history.append(dict(u=u.tolist(),after=x.tolist(),measurement=y.tolist(),estimate=estimate.tolist()))
   error=max(abs(x[6]-vx),max(abs(x[7:11])),abs(x[11]-vx/.05))
   passed=longest>=.03 and clearance>=.015 and pen<=.01 and exc<=.02 and spin<=25 and error<=.3 and landed
   results.append(dict(seed=seed,vx=vx,profileId=id,passed=bool(passed),flightS=longest,clearance=clearance,penetration=pen,jointOverrun=exc,airborneWheelRate=spin,terminalRelativeRate=float(error)))
   traces.append(dict(seed=seed,vx=vx,profileId=id,initial=initial.tolist(),initialEstimate=initialEstimate.tolist(),trace=history))
report=dict(nativeVersion=mujoco.mj_versionString(),results=results)
(ROOT/'test-results/controller-native.json').write_text(json.dumps(dict(report=report,traces=traces)))
print(json.dumps(report));assert all(r['passed'] for r in results)
