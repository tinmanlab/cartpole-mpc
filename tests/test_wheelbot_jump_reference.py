"""Independent native state/force/estimator and jump-acceptance verification.

Replays the actual WASM commands, not the offline designer's ideal state.
No new controller parameters or success thresholds are selected here.
"""
from pathlib import Path
import hashlib
import json
import numpy as np
import mujoco
from scipy.linalg import solve

ROOT=Path(__file__).resolve().parents[1]
sha=lambda name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
p=json.loads((ROOT/'assets/wheelbot/jump_profile.json').read_text())
c=json.loads((ROOT/'tests/fixtures/wheelbot_jump.json').read_text())
raw=json.loads((ROOT/'test-results/jump_final_wasm_raw.json').read_text())
assert mujoco.__version__=='3.7.0'
assert raw['profileSha256']==sha('assets/wheelbot/jump_profile.json')
assert p['assetSha256']==sha('assets/wheelbot/wheelbot.xml')
assert p['baselineSha256']==sha('assets/wheelbot/profile.json')
assert p['recoverySha256']==sha('assets/wheelbot/recovery_profile.json')
assert p['protocolSha256']==raw['protocolSha256']==sha('tests/fixtures/wheelbot_jump.json')
assert p['generatorSha256']==sha('scripts/design_wheelbot_jump.py')
assert p['target']==c['target'] and p['envelope']==c['envelope']
assert p['steps']*p['controlDt']==c['durationSeconds']
assert p['limitsNm']==c['actuatorLimitsNm']
idx=p['controlledIndices'];ref=np.array(p['ref']);U=np.array(p['u']);As=np.array(p['A']);Bs=np.array(p['B']);Ks=np.array(p['K']);Ls=np.array(p['L']);Q=np.array(p['Q']);R=np.array(p['R'])
baseline=json.loads((ROOT/'assets/wheelbot/profile.json').read_text())
for key in ['measurementSigma','Qe','Re','controlledIndices']:assert p[key]==baseline[key]
assert ref.shape==(401,12) and U.shape==(400,3)
assert As.shape==(400,11,11) and Bs.shape==(400,11,3) and Ks.shape==(400,3,11) and Ls.shape==(400,11,5)
recovery=json.loads((ROOT/'assets/wheelbot/recovery_profile.json').read_text())
np.testing.assert_array_equal(Q,recovery['Q']);np.testing.assert_array_equal(R,recovery['R'])
P=np.array(recovery['P']);gain_error=0.
for k in range(p['steps']-1,-1,-1):
 A,B=As[k],Bs[k];K=solve(R+B.T@P@B,B.T@P@A,assume_a='pos');gain_error=max(gain_error,float(np.max(np.abs(K-Ks[k]))))
 F=A-B@K;P=Q+K.T@R@K+F.T@P@F;P=(P+P.T)*.5
assert gain_error<1e-8
Pe=np.array(p['P0']);H=np.eye(11)[:5];I=np.eye(11);filter_gain_error=0.
for k,A in enumerate(As):
 prior=A@Pe@A.T+np.array(p['Qe']);S=H@prior@H.T+np.array(p['Re']);L=solve(S,H@prior,assume_a='pos').T
 filter_gain_error=max(filter_gain_error,float(np.max(np.abs(L-Ls[k]))));Pe=(I-L@H)@prior@(I-L@H).T+L@np.array(p['Re'])@L.T;Pe=(Pe+Pe.T)*.5
assert filter_gain_error<1e-8
m=mujoco.MjModel.from_xml_path(str(ROOT/p['asset']));d=mujoco.MjData(m);wheel=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'wheel_visual');mass=float(m.body_mass.sum());limits=np.array(p['limitsNm'])
max_state=0.;max_estimate=0.;max_command=0.;max_com=0.;max_momentum=np.zeros(3);total_steps=0;rows=[];worst_momentum=None
assert [r['seed'] for r in raw['raw'] if r['mode']=='tvlqr_kf']==c['assessmentSeeds']
for run in raw['raw']:
 trace=run['trace'];seed=run['seed'];rng=seed
 def rand():
  nonlocal_dummy=None
  global rng
  rng=(1664525*rng+1013904223)&0xffffffff;return(rng+.5)/4294967296
 def noise_measure(x):return np.array([x[i]+p['measurementSigma'][i]*np.sqrt(-2*np.log(rand()))*np.cos(2*np.pi*rand())for i in range(5)])
 previous=ref[0].copy();estimate=np.r_[noise_measure(previous),np.zeros(6)];telemetry=[];min_floor=1e9;min_self=1e9;max_tau=np.zeros(3)
 for k,s in enumerate(trace):
  assert s['steps']==k+1 and s['externalForce']==s['last']['externalX']==0
  feedback=Ks[k]@(estimate-ref[k,idx]) if run['mode']=='tvlqr_kf' else np.zeros(3)
  expected=np.clip(U[k]-feedback,-limits,limits);actual=np.array(s['last']['u']);max_command=max(max_command,float(np.max(np.abs(expected-actual))));assert np.all(np.abs(actual)<=limits+1e-12)
  max_tau=np.maximum(max_tau,np.abs(actual))
  mujoco.mj_resetData(m,d);d.qpos[:]=previous[:6];d.qvel[:]=previous[6:];d.ctrl[:]=actual
  for _ in range(5):
   momentum_start=np.r_[d.qpos,d.qvel].copy()
   mujoco.mj_forward(m,d);mujoco.mj_subtreeVel(m,d);before=np.array(d.subtree_linvel[1]);ground=np.zeros(3)
   assert not np.any(d.qfrc_applied) and not np.any(d.xfrc_applied)
   for j in range(d.ncon):
    contact=d.contact[j];wrench=np.zeros(6);mujoco.mj_contactForce(m,d,j,wrench)
    world=contact.frame.reshape(3,3).T@wrench[:3]
    ground+=world if contact.geom1==0 else -world
   mujoco.mj_step(m,d);mujoco.mj_forward(m,d);mujoco.mj_subtreeVel(m,d);after=np.array(d.subtree_linvel[1])
   local_residual=np.abs(mass*(after-before)-.002*(ground+[0,0,-mass*9.81]))
   if worst_momentum is None or float(np.max(local_residual))>worst_momentum['norm']:
    worst_momentum={'norm':float(np.max(local_residual)),'state':momentum_start.copy(),'u':actual.copy()}
   max_momentum=np.maximum(max_momentum,local_residual)
   # Check every integration sample, including the geoms omitted from collision dynamics.
   for geom in [1,2,3]:
    rot=d.geom_xmat[geom].reshape(3,3);size=m.geom_size[geom]
    extent=np.abs(rot[2])@size if geom==1 else abs(rot[2,2])*size[1]+size[0]*np.sqrt(max(0,1-rot[2,2]**2))
    min_floor=min(min_floor,float(d.geom_xpos[geom,2]-extent))
   for one,two in [(1,3),(1,4),(2,4)]:min_self=min(min_self,float(mujoco.mj_geomDistance(m,d,one,two,1,None)))
  native=np.r_[d.qpos,d.qvel];truth=np.array(s['truth']);max_state=max(max_state,float(np.max(np.abs(native-truth))))
  np.testing.assert_allclose(noise_measure(truth),s['measurement'],rtol=1e-10,atol=1e-10)
  prediction=ref[k+1,idx]+As[k]@(estimate-ref[k,idx])+Bs[k]@(actual-U[k]);estimate=prediction+Ls[k]@(np.array(s['measurement'])-prediction[:5]);max_estimate=max(max_estimate,float(np.max(np.abs(estimate-s['estimate']))))
  g=s['last']['geometry'];com=np.array(d.subtree_com[1]);max_com=max(max_com,float(np.max(np.abs(com-g['com']))));contacts=sum(wheel in(d.contact[j].geom1,d.contact[j].geom2)for j in range(d.ncon));clear=float(d.geom_xpos[wheel,2]-.05)
  assert contacts==g['wheelContacts'];np.testing.assert_allclose(clear,g['wheelClearanceM'],atol=1e-8)
  telemetry.append((com.copy(),contacts,clear,truth));previous=truth;total_steps+=1
 # Reconstruct acceptance from native-evaluated COM/contact and actual truth states.
 initial=np.array(run['initialTelemetry']['com']);apex=max(t[0][2]-initial[2] for t in telemetry);flight=0;longest=0
 for _,contacts,clear,_ in telemetry:
  flight=flight+1 if contacts==0 and clear>c['target']['wheelClearanceM'] else 0;longest=max(longest,flight)
 tail=telemetry[-50:];errors=np.max(np.abs(np.array([t[3][:5] for t in tail])-ref[0,:5]),axis=0)
 complete=len(trace)==400 and not trace[-1]['failed'];settle=bool(complete and all(t[1]>0 for t in tail) and errors[0]<.015 and errors[2]<.025 and errors[3]<.03 and errors[4]<.03)
 passed=bool(complete and settle and apex>=.01 and longest*.01>=.03 and min_floor>=0 and min_self>=0)
 assert run['result']['passed']==passed
 np.testing.assert_allclose(run['result']['comApexM'],apex,atol=1e-8)
 assert run['result']['longestFlightSeconds']==longest*.01
 rows.append({'mode':run['mode'],'seed':seed,'steps':len(trace),'completed':complete,'passed':passed,'comApexM':float(apex),'flightSeconds':longest*.01,'tailPositionM':float(errors[0]),'tailPitchRad':float(errors[2]),'minBodyFloorClearanceM':min_floor,'minNonadjacentDistanceM':min_self,'peakTorqueNm':max_tau.tolist()})
assert max_state<1e-7,max_state
assert max_estimate<1e-8,max_estimate
assert max_command<1e-7,max_command
assert max_com<1e-8,max_com
# Semi-implicit Euler does not conserve Cartesian COM momentum exactly across
# finite generalized-coordinate steps. Test refinement at the worst observed
# local sample instead of hiding a failing arbitrary 0.01Ns absolute bound.
refinement=[]
for dt in [.002,.001,.0005,.00025]:
 m.opt.timestep=dt;mujoco.mj_resetData(m,d);d.qpos[:]=worst_momentum['state'][:6];d.qvel[:]=worst_momentum['state'][6:];d.ctrl[:]=worst_momentum['u']
 mujoco.mj_forward(m,d);mujoco.mj_subtreeVel(m,d);before=d.subtree_linvel[1].copy();ground=np.zeros(3)
 for j in range(d.ncon):
  contact=d.contact[j];wrench=np.zeros(6);mujoco.mj_contactForce(m,d,j,wrench);world=contact.frame.reshape(3,3).T@wrench[:3];ground+=world if contact.geom1==0 else -world
 mujoco.mj_step(m,d);mujoco.mj_forward(m,d);mujoco.mj_subtreeVel(m,d)
 error=mass*(d.subtree_linvel[1]-before)-dt*(ground+[0,0,-mass*9.81])
 refinement.append({'dt':dt,'residualNs':error.tolist(),'norm':float(np.max(np.abs(error)))})
assert all(refinement[i+1]['norm']<.4*refinement[i]['norm'] for i in range(3)),refinement
m.opt.timestep=.002
causal=[r for r in rows if r['mode']=='tvlqr_kf'];assert len(causal)==3 and all(r['passed'] for r in causal)
report={'schema':'wheelbot-jump-independent-reference/v1','passed':True,'profileSha256':raw['profileSha256'],'assetSha256':p['assetSha256'],'protocolSha256':p['protocolSha256'],'scriptSha256':sha('tests/test_wheelbot_jump_reference.py'),'controlSteps':total_steps,'physicsSteps':total_steps*5,'maxOneStepStateComponentError':max_state,'maxCausalEstimateComponentError':max_estimate,'maxTorqueErrorNm':max_command,'maxComErrorM':max_com,'maxLocalMomentumResidualNs':max_momentum.tolist(),'worstSampleMomentumRefinement':refinement,'momentumScope':'Observed finite-step residual, with decreasing local error under timestep refinement; not exact conservation or global numerical convergence proof','maxScheduledControlGainError':gain_error,'maxScheduledKalmanGainError':filter_gain_error,'rows':rows,'scope':'Same physical model, reserved noise realizations for this fixed jump. No40Nrecovery,globalrobustness,sensor-onlylocalization or hardware claim.'}
(ROOT/'evidence/wheelbot_jump_reference.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
