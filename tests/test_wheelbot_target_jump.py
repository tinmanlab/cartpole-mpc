"""Independent native KF rollout; all physical checks sampled at 2 ms."""
from pathlib import Path
import sys,json,hashlib,time,importlib.metadata
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1]
p=json.loads((ROOT/'assets/wheelbot/target_jump.json').read_text());base=json.loads((ROOT/'assets/wheelbot/live_profile.json').read_text())
expected=json.loads((ROOT/'package.json').read_text())['dependencies']['@mujoco/mujoco']
assert mujoco.mj_versionString()==mujoco.__version__==importlib.metadata.version('mujoco')==expected==p['nativeVersion'], 'Actual native engine/generation version disagreement; rebuild, never relabel'
fixture=json.loads((ROOT/'tests/fixtures/wheelbot_target_jump.json').read_text());assert p['protocol']==fixture['protocol']
assert p['assetSha256']==hashlib.sha256((ROOT/'assets/wheelbot/live_model.xml').read_bytes()).hexdigest()
assert p['baselineSha256']==hashlib.sha256((ROOT/'assets/wheelbot/live_profile.json').read_bytes()).hexdigest()
m=mujoco.MjModel.from_xml_path(str(ROOT/'assets/wheelbot/live_model.xml'));d=mujoco.MjData(m);wheel=m.geom('wheel_visual').id;ix=p['controlledIndices'];mi=p['measurementIndices'];H=np.eye(11)[[0,1,2,3,4,10]]
results=[];traces=[];started=time.perf_counter()
for pr in p['profiles']:
 for seed in p['protocol']['seeds']:
  rng=seed
  def uniform():
   global rng
   rng=(1664525*rng+1013904223)&0xffffffff;return (rng+.5)/4294967296
  def measure(x):return x[mi]+np.array(p['measurementSigma'])*np.array([np.sqrt(-2*np.log(uniform()))*np.cos(2*np.pi*uniform())for _ in mi])
  offset=.31 if seed==7 else -.27;refs=np.array(pr['ref']);x=np.array(pr['entry']);x[0]+=offset;x[6]=.005 if seed==7 else -.005
  estimate=np.r_[measure(x)[:5],np.zeros(6)];origin=float(estimate[0]);refs[:,0]+=origin;initial=x.copy();initialEstimate=estimate.copy();history=[];path=[];flight=longest=pen=exc=pitch=spin=work=0.;landing=None;landSpeed=None;wasAir=False;peak=np.zeros(3);tail=[]
  for k in range(600):
   terminal=k>=pr.get('terminalStep',140);t=pr['terminal'];r=np.r_[t['qref'][:5],np.zeros(6)] if terminal else refs[k,ix].copy()
   if terminal:r[0]+=origin
   nextRef=r if terminal else refs[k+1,ix];feed=np.array(t['uref'] if terminal else pr['u'][k]);K=np.array(t['K'] if terminal else pr['K'][k]);A=np.array(t['A']if terminal else pr['A'][k]);B=np.array(t['B']if terminal else pr['B'][k]);L=np.array(t['L']if terminal else pr['L'][k]);u=np.clip(feed-K@(estimate-r),-np.array(p['limitsNm']),p['limitsNm']);before=x.copy()
   mujoco.mj_resetData(m,d);d.qpos[:]=x[:6];d.qvel[:]=x[6:];d.ctrl[:]=u
   for _ in range(5):
    mujoco.mj_step(m,d);mujoco.mj_forward(m,d);x=np.r_[d.qpos,d.qvel].copy();pos=d.geom_xpos[wheel,[0,2]].copy();path.append(pos.tolist());contacts=any(wheel in(c.geom1,c.geom2)for c in d.contact);air=not contacts and pos[1]>.055
    flight=flight+.002 if air else 0.;longest=max(longest,flight)
    if air:wasAir=True;spin=max(spin,abs(sum(x[8:12])));work+=abs(u[2]*x[11])*.002
    elif wasAir and landing is None and contacts:
     landing=pos.tolist();jac=np.zeros((3,6));mujoco.mj_jacGeom(m,d,jac,None,wheel);landSpeed=float((jac@x[6:])[0]-.05*sum(x[8:12]))
    pen=max(pen,max([0.]+[-float(c.dist)for c in d.contact]));exc=max(exc,0,abs(x[3])-1.26,abs(x[4])-2.51);pitch=max(pitch,abs(x[2]));assert not d.qfrc_applied.any() and not d.xfrc_applied.any()
   y=measure(x);pred=nextRef+A@(estimate-r)+B@(u-feed);estimate=pred+L@(y-H@pred);peak=np.maximum(peak,abs(u));tail.append(x.tolist());history.append(dict(before=before.tolist(),u=u.tolist(),after=x.tolist(),measurement=y.tolist(),estimate=estimate.tolist()))
   if not np.isfinite(x).all() or abs(x[2])>.6 or x[1]<.15:break
  apex=max(path,key=lambda v:v[1]);accepted=np.array(pr['referenceMetrics']['apex'])+[origin,0];acceptedLand=np.array(pr['referenceMetrics']['landing'])+[origin,0];apexError=float(np.linalg.norm(apex-accepted));landError=float(np.linalg.norm(np.array(landing)-acceptedLand))if landing else 100.;terminalRate=float(np.max(np.abs(np.array(tail)[-50:,6:])))
  checks=dict(landingSpeed=landSpeed is not None and abs(landSpeed)<=p['protocol']['maxLandingTangentialSpeed'],duration=len(history)==600,flight=longest>=.03,clearance=apex[1]>=.07,apex=apexError<=.03,landing=landError<=.03,penetration=pen<=.01,joints=exc<=.02,pitch=pitch<=.6,terminal=terminalRate<=.3,wheelRate=spin<=25)
  checks={k:bool(v)for k,v in checks.items()}
  result=dict(profileId=pr['id'],seed=seed,passed=all(checks.values()),checks=checks,flightS=longest,apex=apex,landing=landing,apexError=apexError,landingError=landError,penetration=pen,jointOverrun=exc,pitch=pitch,terminalRate=terminalRate,airborneWheelRate=spin,airborneAbsoluteMotorWorkJ=work,landingTangentialSpeed=landSpeed,peakTorque=peak.tolist())
  results.append(result);traces.append(dict(profileId=pr['id'],seed=seed,offset=offset,initial=initial.tolist(),initialEstimate=initialEstimate.tolist(),trace=history));print(json.dumps(result),flush=True)
report=dict(schema='wheelbot-target-jump-evidence/v1',assetSha256=p['assetSha256'],profileSha256=hashlib.sha256((ROOT/'assets/wheelbot/target_jump.json').read_bytes()).hexdigest(),nativeVersion=mujoco.mj_versionString(),interpreter=sys.executable,manifestSha256=hashlib.sha256((ROOT/'vendor/manifest.json').read_bytes()).hexdigest(),distributionVersion=importlib.metadata.version('mujoco'),protocol=p['protocol'],archivedAttempts=fixture['archivedAttempts'],results=results,passed=sum(r['passed']for r in results),trials=len(results),seconds=time.perf_counter()-started,scope='Native six noisy measurement KF; translated current x and +/- .005 initial vx; no runtime truth feedback, reset or root force; WASM separate')
(ROOT/'evidence/wheelbot_target_jump.json').write_text(json.dumps(report,indent=2)+'\n');(ROOT/'test-results/target-jump-native.json').write_text(json.dumps(dict(report=report,traces=traces),separators=(',',':'))+'\n')
assert report['passed']==report['trials'], 'Failed trials preserved in evidence'
