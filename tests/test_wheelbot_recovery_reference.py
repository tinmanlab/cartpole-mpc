"""Independent model, same-data KF, native plant and contact audit.

Consumes the locked 72-run assessment. No parameter selection is performed here.
True contact forces are diagnostics only and never controller inputs.
"""
from pathlib import Path
import hashlib
import json

import mujoco
import numpy as np
from scipy.linalg import solve_discrete_are

ROOT = Path(__file__).resolve().parents[1]
read = lambda name: json.loads((ROOT / name).read_text())
sha = lambda name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
base = read('assets/wheelbot/profile.json')
recovery = read('assets/wheelbot/recovery_profile.json')
protocol = read('tests/fixtures/wheelbot_recovery_validation.json')
design = read('evidence/wheelbot_recovery_design.json')
manifest = read('tests/fixtures/wheelbot_cases.json')
raw = read('test-results/wheelbot_mpc_recovery_traces.json')
summary = read('evidence/wheelbot_mpc_recovery_validation.json')
assert sha('assets/wheelbot/profile.json') == 'd54d98ec5b18d89bdf4043a408ea49d7636edcb9c29301b88f446cda9a968e77'
assert sha('assets/wheelbot/response_profile.json') == 'dec4b023f8f54fe8d9de8871104002554b26126e9d7464de8949e9fb3dcbe11d'
assert design['profileSha256'] == sha('assets/wheelbot/recovery_profile.json')
assert design['protocolSha256'] == sha('tests/fixtures/wheelbot_recovery_validation.json')
assert design['designScriptSha256'] == sha('scripts/design_wheelbot_recovery.py')
assert design['accepted'] and design['nonlinearAssessmentInputs'] is False
allowed = {'Q', 'P', 'K', 'dareNormalizedResidual', 'closedLoopRadius', 'responseDesign'}
assert {k:v for k,v in recovery.items() if k not in allowed} == {k:v for k,v in base.items() if k not in allowed}
A,B,Q,R,L = [np.array(recovery[k]) for k in ('A','B','Q','R','L')]
expected_Q = np.array(base['Q']); expected_Q[0,0] *= design['factor']
np.testing.assert_allclose(Q,expected_Q,rtol=1e-13,atol=1e-13)
P = solve_discrete_are(A,B,Q,R)
K = np.linalg.solve(R+B.T@P@B,B.T@P@A)
np.testing.assert_allclose(P,recovery['P'],rtol=1e-9,atol=1e-9)
np.testing.assert_allclose(K,recovery['K'],rtol=1e-9,atol=1e-9)
H=np.eye(11)[:5]; LH=L@H
# Reconstruct the JOINT plant/filter recurrence independently in 22 dimensions.
F=np.block([[A,-B@K],[LH@A,(np.eye(11)-LH)@A-B@K]])
D=np.array(design['disturbanceColumnPerN']); disturbance=np.r_[D,LH@D]
spec=protocol['design']; nominal={}
for name,goal,force in [('position',spec['positionStepM'],0.),('push',0.,spec['pulseForceN'])]:
 z=np.zeros(22);target=np.zeros(11);target[0]=goal;reference=np.r_[B@K@target,B@K@target];tail=[]
 for sample in range(301):
  if sample>=250:tail.append([abs(z[0]-goal),abs(z[2])])
  amplitude=force if 100<=sample<110 else 0.
  z=F@z+reference+disturbance*amplitude
 maxima=np.max(tail,axis=0)
 np.testing.assert_allclose(maxima,[design['modelCases'][name]['positionTailMaxM'],design['modelCases'][name]['pitchTailMaxRad']],rtol=1e-8,atol=1e-10)
 assert maxima[0]<=spec['positionTargetM']+1e-9 and maxima[1]<=spec['pitchTargetRad']+1e-9
 nominal[name]=maxima.tolist()

profiles={'baseline':base,'response':read('assets/wheelbot/response_profile.json'),'recovery':recovery}
assert raw['profileSha256']==summary['profileSha256']=={name:sha('assets/wheelbot/'+('profile' if name=='baseline' else name+'_profile')+'.json') for name in profiles}
key=lambda row:tuple(row[k] for k in ('profile','mode','name','seed'))
raw_rows={key(row):row for row in raw['rows']};summary_rows={key(row):row for row in summary['rows']}
assert len(raw_rows)==len(raw['rows'])==len(summary_rows)==72
assert raw_rows.keys()==summary_rows.keys()
assert {key[3] for key in raw_rows}==set(protocol['assessmentSeeds'])=={401,409,419}
assert {key[0] for key in raw_rows}==set(profiles)
assert {key[1] for key in raw_rows}=={'lqr_kf','mpc_kf'}
model=mujoco.MjModel.from_xml_path(str(ROOT/base['asset'])); data=mujoco.MjData(model)
wheel=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'wheel_visual')
indices=base['controlledIndices']; qref=np.array(base['qref']);uref=np.array(base['uref']);reference=np.r_[qref,np.zeros(6)][indices]
limits=np.array(base['limitsNm']);physical_max=0.;observer_max=0.;same_estimate_max=0.;frames=0;contact_rows=[]

def initial_estimate(initial,seed):
 rng=seed
 def uniform():
  nonlocal rng
  rng=(1664525*rng+1013904223)&0xffffffff
  return (rng+.5)/4294967296
 values=[initial[i]+base['measurementSigma'][i]*np.sqrt(-2*np.log(uniform()))*np.cos(2*np.pi*uniform()) for i in range(5)]
 return np.r_[values,np.zeros(6)]

for trial_key,trial in raw_rows.items():
 row=summary_rows[trial_key];trace=trial['trace'];p=profiles[trial['profile']];gain=np.array(p['K'])
 case=next(c for c in manifest['cases'] if c['name']==trial['name'])
 initial=np.r_[qref,np.zeros(6)];initial[2]+=case['pitchOffset']
 np.testing.assert_allclose(trial['initial'],initial,rtol=0,atol=1e-14)
 estimate=initial_estimate(initial,trial['seed']);previous=initial.copy();details=[]
 assert trace and len(trace)<=manifest['steps']
 for i,s in enumerate(trace):
  assert s['steps']==i+1
  force=case.get('push'); external=force['force'] if force and force['start']<=i<force['end'] else 0.
  assert s['last']['externalX']==external
  u=np.array(s['last']['u']);assert u.shape==(3,) and np.isfinite(u).all() and np.all(np.abs(u)<=limits+1e-8)
  target=reference.copy();target[0]+=case['goal']
  direct=np.clip(uref-gain@(estimate-target),-limits,limits)
  same_estimate_max=max(same_estimate_max,float(np.max(np.abs(direct-s['last']['sameEstimateLqrU']))))
  prediction=reference+A@(estimate-reference)+B@(u-uref)
  estimate=prediction+L@(np.array(s['last']['measurement'])-prediction[:5])
  observer_max=max(observer_max,float(np.max(np.abs(estimate-s['estimate']))))
  # Same canonical MJCF with native integration; one-step replay uses prior raw state.
  mujoco.mj_resetData(model,data);data.qpos[:]=previous[:6];data.qvel[:]=previous[6:];data.ctrl[:]=u;data.qfrc_applied[0]=external
  for _ in range(5):mujoco.mj_step(model,data)
  native=np.r_[data.qpos,data.qvel];truth=np.array(s['truth']);physical_max=max(physical_max,float(np.max(np.abs(native-truth))))
  if trial['mode']=='mpc_kf' and trial['seed']==401 and trial['name'] in ('push','boundary_push'):
   mujoco.mj_forward(model,data);normal=0.;force_x=0.;contacts=0;distance=None
   for j in range(data.ncon):
    c=data.contact[j]
    if wheel not in (c.geom1,c.geom2):continue
    contacts+=1;distance=float(c.dist) if distance is None else min(distance,float(c.dist))
    wrench=np.zeros(6);mujoco.mj_contactForce(model,data,j,wrench)
    assert wrench[0]>=-1e-8
    normal+=float(wrench[0]);world=c.frame.reshape(3,3).T@wrench[:3]
    force_x+=float(world[0])*(1 if c.geom2==wheel else -1)
   details.append({'step':i+1,'forceN':external,'contacts':contacts,'normalN':normal,'contactForceXN':force_x,'distanceM':distance,'pitchRad':float(truth[2]-qref[2]),'estimatedPitchRad':float(estimate[2]-qref[2]),'pitchRateRadS':float(truth[8]),'estimatedPitchRateRadS':float(estimate[7]),'appliedTorqueNm':u.tolist(),'oneStepPredictionDefect':(truth[indices]-prediction).tolist(),'failed':s['failed']})
  previous=truth;frames+=1
 tail=trace[-manifest['tailSteps']:];completed=len(trace)==manifest['steps'] and not trace[-1]['failed'] and trial['rejection'] is None
 passed=completed and all(abs(s['truth'][0]-qref[0]-case['goal'])<manifest['positionTolerance'] and abs(s['truth'][2]-qref[2])<manifest['pitchTolerance'] for s in tail)
 assert row['completed']==completed and row['taskPassed']==passed
 assert row['steps']==len(trace) and row['physicalEnvelopeFailure']==trace[-1]['failed']
 assert row['qpRejected']==(trial['rejection'] is not None)
 assert row['contactLoss']==sum(not s['last']['contact']['wheelContacts'] for s in trace)
 assert row['saturations']==sum(s['last']['saturated'] for s in trace)
 assert row['predictedConstraintActiveSamples']==(sum(s['last']['forecastConstraintActive'] for s in trace) if trial['mode']=='mpc_kf' else None)
 np.testing.assert_allclose(row['tailPositionMax'],max(abs(s['truth'][0]-qref[0]-case['goal']) for s in tail),rtol=1e-12,atol=1e-12)
 np.testing.assert_allclose(row['tailPitchMax'],max(abs(s['truth'][2]-qref[2]) for s in tail),rtol=1e-12,atol=1e-12)
 truths=np.array([s['truth'] for s in trace]);estimates=np.array([s['estimate'] for s in trace]);torques=np.array([s['last']['u'] for s in trace])
 np.testing.assert_allclose(row['estimationRmseByState'],np.sqrt(np.mean((truths[:,indices]-estimates)**2,axis=0)),rtol=1e-10,atol=1e-10)
 np.testing.assert_allclose(row['torqueRmsNm'],np.sqrt(np.mean(torques**2,axis=0)),rtol=1e-10,atol=1e-10)
 np.testing.assert_allclose(row['slewRmsNmPerSecond'],np.sqrt(np.mean((np.diff(np.vstack([uref,torques]),axis=0)/.01)**2,axis=0)),rtol=1e-10,atol=1e-10)
 if details:
  contact_rows.append({'profile':trial['profile'],'case':trial['name'],'seed':401,'completed':completed,'taskPassed':passed,'firstContactLoss':next((s['step'] for s in details if not s['contacts']),None),'firstNormalForceLoss':next((s['step'] for s in details if s['normalN']<1e-4),None),'firstEnvelopeFailure':next((s['step'] for s in details if s['failed']),None),'onset':details[98:115]})
assert physical_max<1e-8,physical_max
assert observer_max<1e-9,observer_max
assert same_estimate_max<1e-9,same_estimate_max
counts=[]
for name in profiles:
 for mode in ('lqr_kf','mpc_kf'):
  rr=[r for r in summary['rows'] if r['profile']==name and r['mode']==mode]
  counts.append({'profile':name,'mode':mode,'completed':sum(r['completed'] for r in rr),'passed':sum(r['taskPassed'] for r in rr),'trials':len(rr),'physicalFailures':sum(r['physicalEnvelopeFailure'] for r in rr),'qpRejected':sum(r['qpRejected'] for r in rr),'cases':{case:sum(r['taskPassed'] for r in rr if r['name']==case) for case in ('local_balance','position','push','boundary_push')}})
report={'schema':'wheelbot-recovery-independent-reference/v1','passed':True,'trials':len(raw_rows),'frames':frames,'profileSha256':design['profileSha256'],'jointModelTailMaxima':nominal,'maxNativeOneStepError':physical_max,'maxObserverMeanError':observer_max,'maxSameEstimateLqrError':same_estimate_max,'counts':counts,'scope':'New noise realizations on known physical cases. No physical OOD, global recovery, or hardware claim.'}
(ROOT/'evidence/wheelbot_recovery_reference.json').write_text(json.dumps(report,indent=2)+'\n')
contact={'schema':'wheelbot-contact-diagnosis/v1','source':'Native MuJoCo replay of actual commands from frozen noisy assessment; true contacts are diagnostic only','normalForceConvention':'Contact-frame first axis is normal; frame.T rotates local force to world; sign is force on wheel geom','totalMassKg':float(model.body_mass.sum()),'gravityMS2':float(-model.opt.gravity[2]),'staticHorizontalFrictionScaleN':float(model.body_mass.sum()*-model.opt.gravity[2]),'staticScaleIsDynamicFeasibilityProof':False,'rows':contact_rows,'conclusion':'The large pulse exits the fixed-contact local-model regime. This does not prove failure for every controller. No contact-loss recovery is implemented or counted as successful.','sources':['https://mujoco.readthedocs.io/en/stable/computation/#contact']}
(ROOT/'evidence/wheelbot_contact_diagnosis.json').write_text(json.dumps(contact,indent=2)+'\n')
print(json.dumps(report,indent=2))
