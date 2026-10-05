"""Independent reconstruction of force selection, actual pulses and physical results.
The new challenge is not re-labelled as a solution of the historical 40N task.
"""
from pathlib import Path
import hashlib,json
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1]
read=lambda f:json.loads((ROOT/f).read_text())
sha=lambda f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest()
c=read('tests/fixtures/wheelbot_force_envelope.json');p=read(c['profile']);r=read('evidence/wheelbot_force_envelope.json');raw=read('test-results/wheelbot_force_raw.json')
assert all(sha(path)==digest for path,digest in r['sourceSha256'].items())
assert raw['sourceSha256']==r['sourceSha256']
assert r['evaluationCount']==len(raw['rows'])==160 and r['lockAtEvaluation']==120
assert not set(c['screenSeeds'])&set(c['assessmentSeeds'])
A,B,K,L=[np.array(p[k]) for k in ['A','B','K','L']];indices=p['controlledIndices'];ref=np.r_[p['qref'],np.zeros(6)];rr=ref[indices];u0=np.array(p['uref']);limits=np.array(p['limitsNm'])
m=mujoco.MjModel.from_xml_path(str(ROOT/c['asset']));d=mujoco.MjData(m);wheel=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'wheel_visual')
assert abs(m.body_mass.sum()-r['massKg'])<1e-12 and m.opt.gravity[2]==-r['gravityMS2']
assert m.opt.timestep==.002 and p['controlDt']==.01
frame_count=0;max_native=0.;max_filter=0.;checked=[]
key=lambda z:(z['amplitudeN'],z['mode'],z['seed'],z['direction'])
for run in raw['rows']:
 phase=run['phase'];trace=run['trace'];previous=ref.copy();rng=run['seed']
 def rand():
  global rng
  rng=(1664525*rng+1013904223)&0xffffffff;return(rng+.5)/4294967296
 def measure(x):return np.array([x[i]+p['measurementSigma'][i]*np.sqrt(-2*np.log(rand()))*np.cos(2*np.pi*rand()) for i in range(5)])
 estimate=np.r_[measure(previous),np.zeros(6)];loss=0;impulse=0.;failed=False
 for k,s in enumerate(trace):
  assert s['steps']==k+1
  expected_force=run['amplitudeN']*run['direction'] if c['pulseStart']<=k<c['pulseStart']+c['pulseSteps'] else 0
  assert s['externalX']==expected_force;impulse+=expected_force*.01
  u=np.array(s['u']);assert np.all(np.abs(u)<=limits+1e-8)
  if run['mode']=='lqr_kf':np.testing.assert_allclose(u,np.clip(u0-K@(estimate-rr),-limits,limits),atol=1e-8)
  mujoco.mj_resetData(m,d);d.qpos[:]=previous[:6];d.qvel[:]=previous[6:];d.ctrl[:]=u;d.qfrc_applied[0]=expected_force
  for _ in range(5):mujoco.mj_step(m,d)
  native=np.r_[d.qpos,d.qvel];truth=np.array(s['truth']);max_native=max(max_native,float(np.max(np.abs(native-truth))))
  np.testing.assert_allclose(s['measurement'],measure(truth),atol=1e-10)
  prediction=rr+A@(estimate-rr)+B@(u-u0);estimate=prediction+L@(np.array(s['measurement'])-prediction[:5]);max_filter=max(max_filter,float(np.max(np.abs(estimate-s['estimate']))))
  mujoco.mj_forward(m,d);contact=sum(wheel in(d.contact[j].geom1,d.contact[j].geom2)for j in range(d.ncon))
  assert contact==s['contact']['wheelContacts'];loss+=contact==0
  failed=bool(abs(truth[0]-ref[0])>1 or abs(truth[2]-ref[2])>.6 or truth[1]<.12);assert s['failed']==failed
  previous=truth;frame_count+=1
 completed=len(trace)==c['steps'] and not failed and not run['qpRejected']
 tail=trace[-c['task']['tailSteps']:]
 xp=max(abs(s['truth'][0]-ref[0])for s in tail) if tail else None
 tp=max(abs(s['truth'][2]-ref[2])for s in tail) if tail else None
 passed=bool(completed and len(tail)==50 and xp<.015 and tp<.025)
 normal=passed and loss==0
 assert run['completed']==completed and run['taskPassed']==passed and run['normalPassed']==normal and run['contactLoss']==loss
 assert run['steps']==len(trace) and abs(run['appliedSignedImpulseNs']-impulse)<1e-12
 if tail:np.testing.assert_allclose([run['tailPositionMax'],run['tailPitchMax']],[xp,tp],atol=1e-12)
 checked.append({**{k:run[k] for k in ['amplitudeN','mode','seed','direction','phase']},'normalPassed':normal,'completed':completed,'taskPassed':passed})
assert max_native<1e-8,max_native
assert max_filter<1e-8,max_filter
screen=[z for z in checked if z['phase']=='screen'];assert len(screen)==120
assert len({key(z)for z in screen})==120
assert all(z['normalPassed']for z in screen if z['amplitudeN']==0)
upper=0
for amp in c['amplitudesN']:
 rows=[z for z in screen if z['amplitudeN']==amp];assert len(rows)==12
 if not all(z['normalPassed']for z in rows):break
 upper=amp
normal=round(np.floor((upper*.8+1e-10)/.1)*.1,1) if upper else None
assert upper==r['selection']['screenUpperN'] and normal==r['selection']['normalAmplitudeN']
assessment=[z for z in checked if z['phase']=='normal-assessment'];stress=[z for z in checked if z['phase']=='stress-assessment']
assert len(assessment)==len(stress)==20
assert {z['seed']for z in assessment}==set(c['assessmentSeeds'])
assert all(z['amplitudeN']==normal for z in assessment) and all(z['amplitudeN']==40 for z in stress)
assert r['admission']['accepted']==all(z['normalPassed']for z in assessment)
assert r['normal']['durationSeconds']==r['stress']['durationSeconds']==.2
assert abs(r['normal']['impulseNs']-normal*.2)<1e-12
assert abs(r['normal']['forceToWeight']-normal/(m.body_mass.sum()*9.81))<1e-12
report={'schema':'wheelbot-force-envelope-reference/v1','passed':True,'sourceEvidenceSha256':sha('evidence/wheelbot_force_envelope.json'),'scriptSha256':sha('tests/test_wheelbot_force_reference.py'),'trials':len(checked),'frames':frame_count,'maximumNativeOneStepError':max_native,'maximumKfMeanError':max_filter,'selectedForceN':normal,'screenUpperN':upper,'normalPassed':sum(z['normalPassed']for z in assessment),'normalTrials':len(assessment),'stressPassed':sum(z['taskPassed']for z in stress),'stressTrials':len(stress),'selectionRecomputed':True,'pulseTimingAndSignsRecomputed':True,'scope':c['claimBoundary']}
(ROOT/'evidence/wheelbot_force_reference.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
